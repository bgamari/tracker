#include <errno.h>
#include <sys/times.h>
#include <sys/stat.h>
#include <sys/types.h>

/* errno definition */
#undef errno
extern int errno;

extern "C" {

caddr_t _sbrk(int incr) {
    return 0;
}

int _close(int file) {
    return -1;
}

int _execve(char *name, char **argv, char **env) {
    errno = ENOMEM;
    return -1;
}

void _exit(int a) {
    while (1);
}

int _fork() {
    errno = EAGAIN;
    return -1;
}

int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _getpid() {
    return 1;
}

int _kill(int pid, int sig) {
    errno = EINVAL;
    return(-1);
}

int isatty(int fildes) {
    return 1;
}

int _isatty(int fildes) {
    return 1;
}

int _link(char *old, char *new_) {
    errno = EMLINK;
    return -1;
}

int _lseek(int file, int ptr, int dir) {
    return 0;
}

int _open(const char *name, int flags, int mode) {
    return -1;
}

int _read(int file, char *ptr, int len) {
    return 0;
}

int _stat(char *file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _times(struct tms *buf) {
    return -1;
}

int _unlink(char *name) {
    errno = ENOENT;
    return -1;
}

int _wait(int *status) {
    errno = ECHILD;
    return -1;
}

int _write(int file, char *ptr, int len) {
    return -1;
}
// vim:expandtab:smartindent:tabstop=4:softtabstop=4:shiftwidth=4:

}
