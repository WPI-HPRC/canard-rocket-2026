#include <sys/stat.h>

#include <errno.h>

extern "C" int _open(const char *name, int flags, int mode) {

    errno = ENOSYS;

    return -1;

}

extern "C" int _close(int file) { return -1; }
extern "C" int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}
extern "C" int _isatty(int file) { return 1; }
extern "C" int _lseek(int file, int ptr, int dir) { return 0; }
extern "C" int _read(int file, char *ptr, int len) { return 0; }
extern "C" int _write(int file, char *ptr, int len) { return len; }