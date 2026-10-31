/* Link-time printf sink for QUIET=1 builds.
 *
 * The normal build keeps the existing polled UART implementation untouched.
 * A quiet build links with --wrap=printf, so calls never enter formatting or
 * _write and therefore cannot delay the cart/USB service loop.
 */
#ifdef RISKY_QUIET
#include <stdarg.h>

int __wrap_printf (const char *format, ...) {
    (void)format;
    return 0;
}
#endif
