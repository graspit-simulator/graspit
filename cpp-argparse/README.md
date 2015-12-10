cpp-optparse

This is yet another option parser for C++. It is modelled after the
excellent Python optparse API. Although incomplete, anyone familiar to 
optparse should feel at home:
http://docs.python.org/library/optparse.html

Copyright (C) 2010 Johannes Weißl <jargon@molb.org>
License: your favourite BSD-style license

Design decisions:
- elegant and easy usage more important than speed / flexibility
- shortness more important than feature completeness
  * no unicode
  * no checking for user programming errors

Why not use getopt/getopt_long?
- not C++ / not completely POSIX
- too cumbersome to use, would need lot of additional code

Why not use Boost.Program_options?
- boost not installed on all target platforms (esp. cluster, HPC, ...)
- too big to include just for option handling:
  322 *.h (44750 lines) + 7 *.cpp (2078 lines)

Why not use tclap/Opag/Options/CmdLine/Anyoption/Argument_helper/...?
- no reason, writing one is faster than code inspection :-)
- similarity to Python desired for faster learning curve

Future work:
- nargs > 1?
- comments?


Example:

    using optparse::OptionParser;
    
    OptionParser parser = OptionParser() .description("just an example");
    
    parser.add_option("-f", "--file").dest("filename")
          .help("write report to FILE").metavar("FILE");
    parser.add_option("-q", "--quiet")
          .action("store_false") 
          .dest("verbose")
          .set_default("1")
          .help("don't print status messages to stdout");
    
    optparse::Values options = parser.parse_args(argc, argv);
    vector<string> args = parser.args();
    
    if (options.get("verbose"))
        cout << options["filename"] << endl;
