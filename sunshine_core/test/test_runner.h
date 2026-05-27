#pragma once
#include <stdio.h>
#include <math.h>
static int _pass = 0, _fail = 0;
#define ASSERT(cond, msg) \
    do { if(cond){_pass++;printf("  ok  %s\n",(msg));} \
         else{_fail++;printf(" FAIL %s  [%s:%d]\n",(msg),__FILE__,__LINE__);} } while(0)
#define ASSERT_NEAR(a, b, tol, msg) \
    do { float _an = (float)(a), _bn = (float)(b), _tn = (float)(tol); \
         ASSERT(fabsf(_an - _bn) < _tn, msg); } while(0)
#define ASSERT_EQ(a, b, msg) \
    do { __typeof__(a) _ae = (a), _be = (__typeof__(a))(b); \
         ASSERT(_ae == _be, msg); } while(0)
#define TEST_RESULTS() \
    do { printf("\n%d passed, %d failed\n", _pass, _fail); \
         return _fail > 0 ? 1 : 0; } while(0)
