#include "Text.h"
#include <iostream>

namespace cli
{
#if defined(_MSC_VER) && (defined(UNICODE) || defined(_UNICODE))
std::basic_istream<text_char>& tin  = std::wcin;
std::basic_ostream<text_char>& tout = std::wcout;
#else
std::basic_istream<text_char>& tin  = std::cin;
std::basic_ostream<text_char>& tout = std::cout;
#endif
} // namespace cli
