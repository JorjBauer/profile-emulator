#pragma once
#include <Arduino.h>
#include <SdFat.h>
#include "selector/selector_blobs.h"

/*
 * Boot-time image selector: emulator side.
 *
 * Protocol and rationale are in selector/SELECTOR.md. In brief: the card ROM
 * boots by reading block 0 to $0800 and jumping to $0801, so whoever supplies
 * block 0 supplies the boot program. Before a selection has been made we supply
 * a small loader; it pulls the selector body in from magic blocks, shows a menu,
 * and then re-reads block 0, which by then is the chosen image's REAL boot block.
 * No reboot, and no changes to the card ROM (which has five spare bytes anyway).
 *
 * Magic blocks are READS in $FF00-$FFFF, above any served volume (we cap at
 * $FE00), so they can never collide with data. On a real ProFile they are out of
 * range and the drive errors before seeking, and a real drive serves its own
 * block 0, so none of this engages.
 */

#define SEL_MAX_IMAGES   64          // limited by the $FF00-$FF3F SELECT range
#define SEL_CFG          "profile.cfg"
#define SEL_REC          64          // catalog record: wide enough for a PATH
#define SEL_PER_PAGE     (512 / SEL_REC)

// One catalog path buffer size, used by the struct AND by the scanner that fills
// it, so the two can never drift apart.
#define SEL_PATH_MAX     80

struct SelImage {
  char     path[SEL_PATH_MAX];
  char     name[17];                 // ProDOS volume name, NUL terminated
  uint32_t blocks;
};

extern SdFat sd;

static SelImage selCatalog[SEL_MAX_IMAGES];
static int      selCount        = 0;
static int      selCurrent      = 0;
static bool     selectorArmed   = true;   // serve the synthetic block 0 until a SELECT
static bool     selValid        = false;  // the persisted path resolved to a real image

// ---- companion filenames ---------------------------------------------------
// Derive a sibling file for whatever image is actually open, by swapping the
// extension: "/Games/foo.img" -> "/Games/foo.tag" / "/Games/foo.bak".
//
// This replaces indexing a fixed name table by openedIndex, which the selector
// never sets: every selected image looked for profile32.tag and backed up to
// profile32.bak no matter which image it really was. Same mistake as pairing
// tags by a separate search, one layer up.
//
// Whatever the image's own extension is, the sibling gets .tag / .bak, so this
// works for .hdv and for upper-case names as well. The only collision it cannot
// see is foo.img and foo.hdv side by side in the same directory: they are two
// different volumes that would share one foo.tag and one foo.bak. Don't do that.
static void selSibling(const char *path, const char *ext, char *out, size_t n) {
  strncpy(out, path, n - 1);
  out[n - 1] = 0;
  char *dot = strrchr(out, '.');
  const char *slash = strrchr(out, '/');
  if (dot && (!slash || dot > slash)) *dot = 0;   // only a real extension
  strncat(out, ext, n - strlen(out) - 1);
}

// ---- ProDOS volume validation ---------------------------------------------
// Only real ProDOS volumes are cataloged. The card serves raw blocks, so a
// Macintosh or other foreign image would be listed but could never boot a //e,
// and a stray $Fx byte in block 2 of such an image is enough to fake a name (a
// 1.44 MB Mac image in testing produced the volume name "8"). Requiring the
// entry_length and entries_per_block constants as well makes the test decisive.
static bool selReadVolName(FsFile &f, char *out) {
  uint8_t b[512];
  out[0] = 0;
  if (!f.seek(2ull * 512) || f.read(b, 512) != 512) return false;
  if ((b[4] & 0xF0) != 0xF0) return false;          // volume directory header
  if (b[0x23] != 0x27 || b[0x24] != 0x0D) return false;  // entry_length / per block
  int n = b[4] & 0x0F;
  if (n < 1 || n > 15) return false;
  int j = 0;
  for (int i = 0; i < n; i++) {
    char c = (char)(b[5 + i] & 0x7F);
    if (c < 32 || c >= 127) return false;
    out[j++] = c;
  }
  out[j] = 0;
  return true;
}

// ---- recursive scan --------------------------------------------------------
// Both .img and .hdv are cataloged: they name the same thing, a flat stream of
// 512-byte ProDOS-order blocks, and which one a tool emits is a matter of taste
// (wozit accepts either for any volume bigger than a floppy). Matching is
// case-insensitive because FAT hands back whatever case the file was created
// with, and a card written on an older machine or by a DOS-era tool is commonly
// all upper case: PROFILE32.IMG must be found just as readily as profile32.img.
//
// Note .po is deliberately NOT accepted. It is the same block order, but nothing
// writes a .po bigger than an 800K floppy, so treating one as a hard disk would
// only ever catalog something that cannot be a ProFile volume.
static bool selHasExt(const char *n, const char *ext3) {
  size_t L = strlen(n);
  if (L < 4) return false;
  const char *e = n + L - 4;
  if (e[0] != '.') return false;
  for (int i = 0; i < 3; i++) {
    char c = e[1 + i];
    if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');   // tolower, ASCII only
    if (c != ext3[i]) return false;
  }
  return true;
}

static bool selEndsWithImg(const char *n) {
  return selHasExt(n, "img") || selHasExt(n, "hdv");
}

// A path that does not fit is SKIPPED, never truncated. The selection is
// persisted by path and reopened by path, so a truncated path is not a cosmetic
// problem: it would catalog an entry that cannot be opened on the next boot.
// Two images sharing their first SEL_PATH_MAX-1 characters would also collapse
// into the same string. Persisting by path instead of by index exists to
// prevent that confusion. Report and drop instead.
//
// Same for a filename too long for nm[]: SdFat's getName() truncates without error and
// returns the length it copied, so a copy that exactly fills the buffer is treated
// as suspect.
static void selScanDir(FsFile dir, const char *prefix, int depth) {
  if (depth > 3 || selCount >= SEL_MAX_IMAGES) return;
  FsFile f;
  while (f.openNext(&dir, O_RDONLY)) {
    char nm[64];
    size_t nmLen = f.getName(nm, sizeof(nm));
    if (nmLen == 0 || nmLen >= sizeof(nm) - 1) {   // empty, or truncated by SdFat
      if (nmLen) { Serial.print("selector: name too long, skipped: "); Serial.println(nm); }
      f.close();
      continue;
    }
    if (nm[0] == '.') { f.close(); continue; }     // skip . .. and dotfiles
    // prefix is "" at the root and "/dir" below it, and is never built with a
    // trailing slash, so one separator is always right. The old code indexed
    // prefix[strlen(prefix) - 1] to test for one, which read prefix[-1] on the
    // root call.
    char full[SEL_PATH_MAX];
    int need = snprintf(full, sizeof(full), "%s/%s", prefix, nm);
    if (need < 0 || (size_t)need >= sizeof(full)) {
      Serial.print("selector: path too long, skipped: ");
      Serial.print(prefix); Serial.print("/"); Serial.println(nm);
      f.close();
      continue;
    }
    if (f.isDir()) {
      selScanDir(f, full, depth + 1);
    } else if (selEndsWithImg(nm) && f.fileSize() >= 512ull * 8 && selCount < SEL_MAX_IMAGES) {
      SelImage &e = selCatalog[selCount];
      memcpy(e.path, full, (size_t)need + 1);       // checked above: fits with its NUL
      e.blocks = (uint32_t)(f.fileSize() / 512);
      if (selReadVolName(f, e.name)) selCount++;    // skip anything not ProDOS
    }
    f.close();
  }
}

static void selScan() {
  selCount = 0;
  FsFile root;
  if (!root.open("/", O_RDONLY)) return;
  selScanDir(root, "", 0);
  root.close();
}

// ---- persisted choice ------------------------------------------------------
// The choice is persisted as a PATH, never an index, so the card can be pulled,
// the catalog rearranged, and the same image still selected on return. If the
// path no longer resolves (or there is no config at all) we do NOT invent a
// default and boot it: selValid stays false and the menu is forced.
static void selLoadConfig() {
  selCurrent = 0;
  selValid   = false;
  FsFile c;
  if (!c.open(SEL_CFG, O_RDONLY)) return;
  char line[96];
  int n = c.read(line, sizeof(line) - 1);
  c.close();
  if (n <= 0) return;
  line[n] = 0;
  for (int i = 0; i < n; i++) if (line[i] == '\r' || line[i] == '\n') { line[i] = 0; break; }
  for (int i = 0; i < selCount; i++)
    if (!strcmp(selCatalog[i].path, line)) { selCurrent = i; selValid = true; return; }
}

// Forget the saved selection: the next boot forces the menu. Useful on the bench
// and when handing the card to someone else.
static void selForget() {
  sd.remove(SEL_CFG);
  selValid = false;
}

static void selSaveConfig() {
  if (selCurrent < 0 || selCurrent >= selCount) {
    Serial.println("SAVE SELECTION SKIPPED: no current image");
    return;
  }
  FsFile c;
  if (!c.open(SEL_CFG, O_WRONLY | O_CREAT | O_TRUNC)) {
    // The failure that forces the menu forever. It used to be a bare return,
    // so a card that could not be written looked exactly like a selector that
    // ignored the user.
    Serial.print("SAVE SELECTION FAILED: cannot write "); Serial.println(SEL_CFG);
    return;
  }
  size_t n = strlen(selCatalog[selCurrent].path);
  bool ok = (c.write(selCatalog[selCurrent].path, n) == n) && (c.write("\n", 1) == 1);
  c.sync();
  c.close();
  if (!ok) Serial.println("SAVE SELECTION FAILED: short write");
  else { Serial.print("selection saved: ");
         Serial.println(selCatalog[selCurrent].path); }
  // selValid tracks "a saved choice that resolves", and it has just become true
  // by construction. Without this the force-menu bit stays set for the rest of
  // the session even though a choice HAS been made.
  selValid = ok;
}

// ---- magic blocks ----------------------------------------------------------
// Returns true if `low` (the low byte of a $FFxx block) was a magic block and
// buf has been filled. selectImage() is supplied by the sketch because switching
// images touches the open file and the geometry.
static bool selApplySelection(int idx);          // fwd, defined in the sketch

static bool selMagicBlock(uint8_t low, uint8_t *buf, int bufLen) {
  memset(buf, 0, bufLen);

  if (low <= 0x3F) {                              // $FF00-$FF3F: SELECT
    buf[0] = selApplySelection(low) ? 0x00 : 0xFF;
    return true;
  }
  if (low >= 0x40 && low <= 0x4F) {               // $FF40-$FF4F: selector body
    uint32_t off = (uint32_t)(low - 0x40) * 512;
    if (off < sizeof(SEL_BODY)) {
      uint32_t n = sizeof(SEL_BODY) - off;
      if (n > 512) n = 512;
      memcpy(buf, SEL_BODY + off, n);
    }
    return true;
  }
  if (low >= 0x80 && low <= 0x8F) {               // $FFFF80-$FFFF8F: catalog
    int first = (low - 0x80) * SEL_PER_PAGE;
    for (int i = 0; i < SEL_PER_PAGE; i++) {
      int idx = first + i;
      if (idx >= selCount) break;
      uint8_t *r = buf + i * SEL_REC;
      r[0] = 0x01 | ((selValid && idx == selCurrent) ? 0x02 : 0x00);
      // The PATH, not the volume name: two images can share a volume name 
      // (e.g. "PROFILE"), so the name alone cannot tell them apart. Show
      // the TAIL of a long path, which is the part that distinguishes it.
      memset(r + 2, ' ', 48);
      const char *p = selCatalog[idx].path;
      int L = (int)strlen(p);
      if (L > 48) p += (L - 48);
      // UPPERCASE for display. The //e text screen has no inverse lowercase:
      // screen codes $00-$3F are inverse uppercase and symbols only, so COUT with
      // INVFLG=$3F folds lowercase into digits and the highlighted row renders as
      // garbage. This field is display-only; selection is by index and the real
      // path is kept internally, so case here costs nothing.
      for (int j = 0; j < 48 && p[j]; j++) {
        char c = p[j];
        r[2 + j] = (c >= 'a' && c <= 'z') ? (c - 32) : c;
      }
      r[50] = selCatalog[idx].blocks         & 0xFF;
      r[51] = (selCatalog[idx].blocks >> 8)  & 0xFF;
      r[52] = (selCatalog[idx].blocks >> 16) & 0xFF;
      r[53] = (selCatalog[idx].blocks >> 24) & 0xFF;
    }
    return true;
  }
  if (low == 0xC0) {                              // $FFC0: info
    buf[0] = (uint8_t)selCount;
    buf[1] = (uint8_t)selCurrent;
    buf[2] = (uint8_t)((selCount + SEL_PER_PAGE - 1) / SEL_PER_PAGE);
    buf[3] = (uint8_t)((sizeof(SEL_BODY) + 511) / 512);
    buf[4] = 0xA5;                                // "an emulator is answering"
    buf[5] = selValid ? 0x00 : 0x01;              // bit0: FORCE the menu, whatever
                                                  // open-apple is doing
    return true;
  }
  return false;
}

// True when block 0 should be answered with the synthetic loader rather than the
// image's own boot block: only before a selection, and only when there is an
// actual choice to make.
// Deciding when to serve the synthetic block 0.
//
// A timer was wrong: re-arming after N seconds of idle races a fast reboot, so
// the menu appeared only sometimes. There IS a deterministic signal. The card
// ROM's boot always reads the get-status block $FFFFFF immediately before it
// reads block 0, and nothing else in normal operation does that. So:
//
//   get-status then block 0  -> a BOOT: serve the selector
//   anything else then block 0 -> the selector's own chain, or ProDOS at
//                                 runtime: serve the image's real block 0
//
// No timers and no arming state that can get stuck, and the chain works because
// the read before it is the SELECT block, not the get-status block.
static uint32_t selPrevBlk = 0xFFFFFFFFu;

static inline bool selWantSyntheticBoot() {
  if (selCount < 1) return false;
  return selCount > 1 || !selValid;            // one image AND a valid choice: no menu
}
