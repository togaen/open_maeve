/*
 * Copyright 2019 Maeve Automation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
#pragma once

/** @brief Define a silencer for use when logging is not desired. */
#ifdef LOG_SILENT
static_assert(false, "Macro LOG_SILENT already defined. Cannot continue.");
#else
#define LOG_SILENT(str)
#endif

/**
 * @brief Concatenate preprocessor tokens A and B without expanding macro
 * definitions (however, if invoked from a macro, macro arguments are expanded).
 *
 * @url
 * https://stackoverflow.com/questions/5256313/c-c-macro-string-concatenation
 */
#define PPCAT_NX(A, B) A##B

/**
 * @brief Concatenate preprocessor tokens A and B after macro-expanding them.
 *
 * @url
 * https://stackoverflow.com/questions/5256313/c-c-macro-string-concatenation
 */
#define PPCAT(A, B) PP(A, B)

/** @brief Convenience for printing name/value pairs. */
#define STREAMIZE(var, member) "" << #member << ": " << var.member
