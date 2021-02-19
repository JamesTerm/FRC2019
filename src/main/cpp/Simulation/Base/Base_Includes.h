#pragma once
#include <sys/types.h>

#ifdef __GNUC__
typedef long long __int64;
//make these options stick--- no pop
#pragma GCC diagnostic push
//-Wno-unknown-pragmas
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wreorder"
#else
// No C library depreciation warnings
#pragma warning ( disable : 4995 )
#pragma warning ( disable : 4996 )
#pragma warning ( disable : 4477 )

#ifndef _CRT_SECURE_NO_WARNINGS
    #define _CRT_SECURE_NO_WARNINGS
#endif

#endif

//typedef unsigned long size_t;
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string>
#include <list>
#include <vector>
#include <map>
#include <assert.h>
#include <string.h>
