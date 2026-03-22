// A minimalistic 2D and 3D physics engine
// https://github.com/dmitry-sapelnikov/neat-physics
// SPDX-FileCopyrightText: 2025-2026 Dmitry Sapelnikov
// SPDX-License-Identifier: MIT

#pragma once

// Platform detection
#if defined(_WIN32) || defined(_WIN64)
#define NPH_PLATFORM_WINDOWS
#elif defined(__linux__)
#define NPH_PLATFORM_LINUX
#elif defined(__APPLE__) || defined(__MACH__)
#define NPH_PLATFORM_MACOS
#else
#error "Unsupported platform"
#endif

// Compiler detection
#if defined(_MSC_VER)
#define NPH_COMPILER_MSVC
#elif defined(__GNUC__)
#define NPH_COMPILER_GCC
#elif defined(__clang__)
#define NPH_COMPILER_CLANG
#endif

// Force inline macro
#ifdef NPH_COMPILER_MSVC
#define NPH_FORCE_INLINE __forceinline
#elif defined(NPH_COMPILER_GCC) || defined(NPH_COMPILER_CLANG)
#define NPH_INLINE inline __attribute__((always_inline))
#endif
