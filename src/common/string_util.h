#ifndef _STRING_UTIL_H
#define _STRING_UTIL_H

#include "zarray.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct string_buffer string_buffer_t;

typedef struct string_feeder string_feeder_t;

/**
 * Similar to sprintf(), except that it will malloc() enough space for the
 * formatted string which it returns. It is the caller's responsibility to call
 * free() on the returned string when it is no longer needed.
 */
char *sprintf_alloc(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));

/**
 * Concatenates 1 or more strings together and returns the result, which will be a
 * newly allocated string which it is the caller's responsibility to free.
 */
#define str_concat(...) _str_concat_private(__VA_ARGS__, NULL)
char *_str_concat_private(const char *first, ...);

/**
 * Splits the supplied string into an array of strings by subdividing it at
 * each occurrence of the supplied delimiter string. The split strings will not
 * contain the delimiter. The original string will remain unchanged.
 * If str is composed of all delimiters, an empty array will be returned.
 *
 * It is the caller's responsibilty to free the returned zarray, as well as
 * the strings contained within it, e.g.:
 *
 *   zarray_t *za = str_split("this is a haystack", " ");
 *      => ["this", "is", "a", "haystack"]
 *   zarray_vmap(za, free);
 *   zarray_destroy(za);
 */
zarray_t *str_split(const char *str, const char *delim);

/**
 * Identical to str_split() except that the delimiter can be an extended
 * regular expression, e.g.:
 *   '[[:space:]]+' or '\\s' will tokenize on any whitespace character
 *
 * Note: using an asterisk anywhere in the regex will assume a ^ at the beginning
 *
 * Returns NULL if 'regex' is not a valid regular expression.
 */
zarray_t *str_split_regex(const char *str, const char *regex);

/**
 * Returns an array of all of the substrings within 'str' which match the
 * supplied extended regular expression 'regex'. The original string will remain
 * unchanged, and all strings contained within the array will be newly-allocated.
 *
 * Note: using an asterisk anywhere in the regex will assume a ^ at the beginning
 *
 * It is the caller's responsibilty to free the returned zarray, as well as
 * the strings contained within it, e.g.:
 *
 *   zarray_t *za = str_match_regex("needles in a haystack", "[aeioyu]{2}");
 *      => ["ee", "ay"]
 *   zarray_vmap(za, free);
 *   zarray_destroy(za);
 */
zarray_t *str_match_regex(const char *str, const char *regex);

/**
 * Trims whitespace characters (i.e. matching isspace()) from the beginning and/or
 * end of the supplied string. This change affects the supplied string in-place.
 * The supplied/edited string is returned to enable chained reference.
 *
 * Note: do not pass a string literal to this function
 */
char *str_trim(char *str);

/**
 * Trims whitespace characters (i.e. matching isspace()) from the beginning
 * of the supplied string. This change affects the supplied string in-place.
 * The supplied/edited string is returned to enable chained reference.
 *
 * Note: do not pass a string literal to this function
 */
char *str_lstrip(char *str);

/**
 * Trims whitespace characters (i.e. matching isspace()) from the end of the
 * supplied string. This change affects the supplied string in-place.
 * The supplied/edited string is returned to enable chained reference.
 *
 * Note: do not pass a string literal to this function
 */
char *str_rstrip(char *str);

/**
 * Returns 1 if the end of string 'haystack' matches 'needle', else 0.
 *
 * Note: An empty needle ("") will match any source.
 */
int str_ends_with(const char *haystack, const char *needle);

/**
 * Returns 1 if the start of string 'haystack' matches 'needle', else 0.
 *
 * Note: An empty needle ("") will match any source.
 */
int str_starts_with(const char *haystack, const char *needle);

/**
 * Retrieves a (newly-allocated) substring of the given string, 'str', starting
 * from character index 'startidx' through index 'endidx' - 1 (inclusive).
 * An 'endidx' value -1 is equivalent to strlen(str).
 *
 * It is the caller's responsibility to free the returned string.
 *
 * Examples:
 *   str_substring("string", 1, 3) = "tr"
 *   str_substring("string", 2, -1) = "ring"
 *   str_substring("string", 3, 3) = ""
 *
 * Note: startidx must be >= endidx
 */
char *str_substring(const char *str, size_t startidx, long endidx);

/**
 * Retrieves the zero-based index of the beginning of the supplied substring
 * (needle) within the search string (haystack) if it exists.
 *
 * Returns -1 if the supplied needle is not found within the haystack.
 */
int str_indexof(const char *haystack, const char *needle);

/**
 * Replaces all upper-case characters within the supplied string with their
 * lower-case counterparts, modifying the original string's contents.
 *
 * Returns the supplied / modified string.
 */
char *str_tolowercase(char *s);

/**
 * Replaces all occurrences of 'needle' in the string 'haystack', substituting
 * for them the value of 'replacement', and returns the result as a newly-allocated
 * string. The original strings remain unchanged.
 *
 * It is the caller's responsibility to free the returned string.
 *
 * Examples:
 *   str_replace("string", "ri", "u") = "stung"
 *   str_replace("singing", "ing", "") = "s"
 *   str_replace("string", "foo", "bar") = "string"
 *
 * Note: An empty needle will match only an empty haystack
 */
char *str_replace(const char *haystack, const char *needle, const char *replacement);

//////////////////////////////////////////////////////
// String Buffer

/**
 * Creates and initializes a string buffer object which can be used with any of
 * the string_buffer_*() functions.
 *
 * It is the caller's responsibility to free the string buffer resources with
 * a call to string_buffer_destroy() when it is no longer needed.
 */
string_buffer_t *string_buffer_create();

/**
 * Frees the resources associated with a string buffer object, including space
 * allocated for any appended characters / strings.
 */
void string_buffer_destroy(string_buffer_t *sb);

/**
 * Appends a single character to the end of the supplied string buffer.
 */
void string_buffer_append(string_buffer_t *sb, char c);

/**
 * Appends the supplied string to the end of the supplied string buffer.
 */
void string_buffer_append_string(string_buffer_t *sb, const char *str);

/**
 * Formats the supplied string and arguments in a manner akin to printf(), and
 * appends the resulting string to the end of the supplied string buffer.
 */
void string_buffer_appendf(string_buffer_t *sb, const char *fmt, ...) __attribute__ ((format (printf, 2, 3)));

/**
 * Determines whether the character contents held by the supplied string buffer
 * ends with the supplied string.
 *
 * Returns 1 if the string buffer's contents ends with 'str', else 0.
 */
int string_buffer_ends_with(string_buffer_t *sb, const char *str);

/**
 * Returns the string-length of the contents of the string buffer (not counting \0).
 * Equivalent to calling strlen() on the string returned by string_buffer_to_string(sb).
 */
size_t string_buffer_size(string_buffer_t *sb);

/**
 * Returns the contents of the string buffer in a newly-allocated string, which
 * it is the caller's responsibility to free once it is no longer needed.
 */
char *string_buffer_to_string(string_buffer_t *sb);

/**
 * Clears the contents of the string buffer, setting its length to zero.
 */
void string_buffer_reset(string_buffer_t *sb);

//////////////////////////////////////////////////////
// String Feeder

/**
 * Creates a string feeder object which can be used to traverse the supplied
 * string using the string_feeder_*() functions. A local copy of the string's
 * contents will be stored so that future changes to 'str' will not be
 * reflected by the string feeder object.
 *
 * It is the caller's responsibility to call string_feeder_destroy() on the
 * returned object when it is no longer needed.
 */
string_feeder_t *string_feeder_create(const char *str);

/**
 * Frees resources associated with the supplied string feeder object, after
 * which it will no longer be valid for use.
 */
void string_feeder_destroy(string_feeder_t *sf);

/**
 * Determines whether any characters remain to be retrieved from the string
 * feeder's string (not including the terminating '\0').
 *
 * Returns 1 if at least one more character can be retrieved with calls to
 * string_feeder_next(), string_feeder_peek(), string_feeder_peek(), or
 * string_feeder_consume(), else 0.
 */
int string_feeder_has_next(string_feeder_t *sf);

/**
 * Retrieves the next available character from the supplied string feeder
 * (which may be the terminating '\0' character) and advances the feeder's
 * position to the next character in the string.
 *
 * Note: Attempts to read past the end of the string will throw an assertion.
 */
char string_feeder_next(string_feeder_t *sf);

/**
 * Retrieves a series of characters from the supplied string feeder. The number
 * of characters returned will be 'length' or the number of characters
 * remaining in the string, whichever is shorter. The string feeder's position
 * will be advanced by the number of characters returned.
 *
 * It is the caller's responsibility to free the returned string when it is no
 * longer needed.
 *
 * Note: Calling once the end of the string has already been read will throw an assertion.
 */
char *string_feeder_next_length(string_feeder_t *sf, int length);

/**
 * Retrieves the next available character from the supplied string feeder
 * (which may be the terminating '\0' character), but does not advance
 * the feeder's position so that subsequent calls to _next() or _peek() will
 * retrieve the same character.
 *
 * Note: Attempts to peek past the end of the string will throw an assertion.
 */
char string_feeder_peek(string_feeder_t *sf);

/**
 * Retrieves a series of characters from the supplied string feeder. The number
 * of characters returned will be 'length' or the number of characters
 * remaining in the string, whichever is shorter. The string feeder's position
 * will not be advanced.
 *
 * It is the caller's responsibility to free the returned string when it is no
 * longer needed.
 *
 * Note: Calling once the end of the string has already been read will throw an assertion.
 */
char *string_feeder_peek_length(string_feeder_t *sf, int length);

/**
 * Retrieves the line number of the current position in the supplied
 * string feeder, which will be incremented whenever a newline is consumed.
 *
 * Examples:
 *   prior to reading 1st character:                line = 1, column = 0
 *   after reading 1st non-newline character:       line = 1, column = 1
 *   after reading 2nd non-newline character:       line = 1, column = 2
 *   after reading 1st newline character:           line = 2, column = 0
 *   after reading 1st character after 1st newline: line = 2, column = 1
 *   after reading 2nd newline character:           line = 3, column = 0
 */
int string_feeder_get_line(string_feeder_t *sf);

/**
 * Retrieves the column index in the current line for the current position
 * in the supplied string feeder, which will be incremented with each
 * non-newline character consumed, and reset to 0 whenever a newline (\n) is
 * consumed.
 *
 * Examples:
 *   prior to reading 1st character:                line = 1, column = 0
 *   after reading 1st non-newline character:       line = 1, column = 1
 *   after reading 2nd non-newline character:       line = 1, column = 2
 *   after reading 1st newline character:           line = 2, column = 0
 *   after reading 1st character after 1st newline: line = 2, column = 1
 *   after reading 2nd newline character:           line = 3, column = 0
 */
int string_feeder_get_column(string_feeder_t *sf);

/**
 * Determines whether the supplied string feeder's remaining contents starts
 * with the given string.
 *
 * Returns 1 if the beginning of the string feeder's remaining contents matches
 * the supplied string exactly, else 0.
 */
int string_feeder_starts_with(string_feeder_t *sf, const char *str);

/**
 * Consumes from the string feeder the number of characters contained in the
 * given string (not including the terminating '\0').
 *
 * Throws an assertion if the consumed characters do not exactly match the
 * contents of the supplied string.
 */
void string_feeder_require(string_feeder_t *sf, const char *str);

#ifdef __cplusplus
}
#endif

#endif
