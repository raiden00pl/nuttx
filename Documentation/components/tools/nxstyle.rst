=============
``nxstyle.c``
=============

I am embarrassed that this is here. This program is a complete hack but,
unfortunately, it has become so useful to me that I need to keep it here.

A little background:  I have tinkered with pretty printers for some
time and have not been happy with the results.  An alternative that
occurred to me would be just a standard checker that examines a C
file that gives warnings for violations of the coding standard.

This turns out to be more difficult that you might think. A pretty
printer understands C syntax:  They break the file up into its C
components then reassembles the output in the format. But parsing the
C loses the original file layout and so it not useful in this case.

This program instead, uses a collection of heuristics (i.e., hacks and
bandaids) to examine the C file for obvious violations of the coding
standard.  This program is completely ignorant of C syntax; it simply
performs crude pattern matching to check the file.

Prints formatted messages that are classified as info, warn, error,
fatal. In a parsable format that can be used by editors and IDEs.

Usage::

         nxstyle [-m <excess>] [-v <level>] [-r <start,count>] <filename>
         nxstyle -h this help
         nxstyle -v <level> where level is
                    0 - no output
                    1 - PASS/FAIL
                    2 - output each line (default)

Per-directory configuration
===========================

Place an optional ``.nxstyle`` file in a directory containing C source or
header files that need additional style configuration.  Its settings apply
to files in that directory.

The configuration uses a simple key-value syntax.  The only supported option
is currently ``ignore-words-list``.  It contains a comma-separated list of
mixed-case identifiers that ``nxstyle`` should accept.  A plain entry matches
a complete identifier.  An entry ending in ``*`` matches every identifier
with that prefix.  For example::

  ignore-words-list =
    ThirdParty_*,
    publicMember

This accepts ``ThirdParty_Function`` and ``publicMember``, but it does not
accept ``publicMemberExtra``.  The base of each entry must be a valid C
identifier and ``*`` is only valid as the final character.  Blank lines and
lines beginning with ``#`` or ``;`` are ignored.

Unknown options and unrecognized lines are ignored so that newer
configuration files remain compatible with older versions of ``nxstyle``.
Invalid values for a recognized option are fatal and are reported with the
configuration filename and line number.

See also indent.sh and uncrustify.cfg
