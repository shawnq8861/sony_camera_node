#ifndef TEXT_H
#define TEXT_H

#include <istream>
#include <ostream>
#include <regex>
#include <sstream>
#include <string>

namespace cli
{
/*** Text Character ***/
#if defined(_MSC_VER) && (defined(UNICODE) || defined(_UNICODE))
#define TCHAR wchar_t
#define TEXT(s) L ## s
#else
#define TCHAR char
#define TEXT(s) s
#endif
using text_char = TCHAR;

/*** Text String ***/
using text_literal = text_char const*;
using text = std::basic_string<text_char>;

/*** Text Regex ***/
using tregex = std::basic_regex<text_char>;
using tsmatch = std::match_results<text::const_iterator>;

/*** Text Streams ***/
using text_stringstream = std::basic_stringstream<text_char>;
extern std::basic_istream<text_char>& tin;
extern std::basic_ostream<text_char>& tout;
} // namespace cli

#endif // !TEXT_H
