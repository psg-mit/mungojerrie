/* -*- C++ -*- */
/**
  @file Pscanner.ll

  @brief Scanner definition for PRISM parser.

  @author Mateo Perez, Fabio Somenzi, Ashutosh Trivedi

  @copyright@parblock
  Copyright (c) 2021, Regents of the University of Colorado

  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

  Neither the name of the University of Colorado nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
  @endparblock

*/

%{
#include <cstdlib>
#include <cerrno>
#include <climits>
#include <string>
#include <sstream>
#include "Pwrapper.hh"

/* Work around an incompatibility in flex (at least versions
   2.5.31 through 2.5.33): it generates code that does
   not conform to C89.  See Debian bug 333231
   <http://bugs.debian.org/cgi-bin/bugreport.cgi?bug=333231>.  */
# undef yywrap
# define yywrap() 1

/* By default yylex returns int, we use token_type.
   Unfortunately yyterminate by default returns 0, which is
   not of token_type.  */
#define yyterminate() return token::END
%}

/* the never-interactive option is a hack for cygwin
 * compilation with g++ 4.3.4 and 4.5.3.  It is hoped that
 * it will soon become unnecessary. */
%option noyywrap nounput prefix="p" batch debug never-interactive

inum  [0-9]+
fpnum [0-9]*\.?[0-9]+([eE][\-+]?[0-9]+)?
id    [A-Za-z_][A-Za-z0-9_]*
blank [ \t]
cmnt  \/\/.*

%{
# define YY_USER_ACTION  yylloc->columns(yyleng);
%}
%%
%{
  yylloc->step();
%}
{blank}+   yylloc->step();
[\n]+      yylloc->lines(yyleng); yylloc->step();
{cmnt}     yylloc->step();
        /* Skip DOS cariage return characters. */
[\r]

%{
  typedef pparser::p_parser::token token;
%}
        /* Convert ints to the actual type of tokens.  */
[&|!^+\-*/\?=<>:;(){},\[\]\"\'] return pparser::p_parser::token_type(yytext[0]);
"!="             return token::NOTEQ;
"->"             return token::ARROW;
".."             return token::RANGE;
"<=>"            return token::IFF;
"=>"             return token::IMPLIES;
"<="             return token::LE;
">="             return token::GE;
A                return token::A;
bool             return token::BOOL;
ceil             return token::CEIL;
clock            return token::CLOCK;
const            return token::CONST;
ctmc             return token::CTMC;
C                return token::C;
double           return token::DOUBLE;
dtmc             return token::DTMC;
E                return token::E;
endinit          return token::ENDINIT;
endinvariant     return token::ENDINVARIANT;
endmodule        return token::ENDMODULE;
endplayer        return token::ENDPLAYER;
endrewards       return token::ENDREWARDS;
endsystem        return token::ENDSYSTEM;
false            return token::FALSE;
floor            return token::FLOOR;
formula          return token::FORMULA;
filter           return token::FILTER;
func             return token::FUNC;
F                return token::F;
global           return token::GLOBAL;
G                return token::G;
init             return token::INIT;
invariant        return token::INVARIANT;
I                return token::I;
int              return token::INT;
label            return token::LABEL;
log              return token::LOG;
max              return token::MAX;
mdp              return token::MDP;
min              return token::MIN;
mod              return token::MOD;
module           return token::MODULE;
X                return token::X;
nondeterministic return token::NONDETERMINISTIC;
player           return token::PLAYER;
Pmax             return token::PMAX;
Pmin             return token::PMIN;
P                return token::P;
pow              return token::POW;
probabilistic    return token::PROBABILISTIC;
prob             return token::PROB;
pta              return token::PTA;
rate             return token::RATE;
rewards          return token::REWARDS;
Rmax             return token::RMAX;
Rmin             return token::RMIN;
R                return token::R;
S                return token::S;
smg              return token::SMG;
bdp              return token::BDP;
stochastic       return token::STOCHASTIC;
system           return token::SYSTEM;
true             return token::TRUE;
U                return token::U;
W                return token::W;
{id}       yylval->build<std::string>(yytext); return token::IDENTIFIER;
{inum}     { yylval->build<int>(std::atoi(yytext));
             return token::INUMBER; }
{fpnum}    { yylval->build<double>(std::atof(yytext));
             return token::FPNUMBER; }
.          { driver.error(*yylloc, std::string("invalid character: ") + yytext[0]);
             return token::INVALID_CHAR; }
%%

void p_driver::scan_begin(void)
{
  yy_flex_debug = trace_scanning;
  if (file == "-")
    yyin = stdin;
  else if (!(yyin = fopen(file.c_str(), "r"))) {
    error(std::string("cannot open ") + file);
    exit(1);
  }
}

void p_driver::scan_end(void)
{
  fclose(yyin);
  yylex_destroy();
}
