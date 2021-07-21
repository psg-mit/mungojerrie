#ifndef P_DRIVER_HH_
#define P_DRIVER_HH_

/** @file Pdriver.hh

  @brief Driver for PRISM parser.

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

#include <map>
#include <queue>
#include <string>
#include "Pparser.hh"

/** @brief Scanning and parsing of (restricted) PRISM models. */
class p_driver {
public:
  /// Constructor.
  p_driver(Model &model,
           std::vector<std::pair<std::string,std::string>> const & defines,
           Verbosity::Level verbosity = Verbosity::Silent);

  /// @brief Starts the scanner.
  void scan_begin(void);
  /// @brief Ends the scanner.
  void scan_end(void);

  /// @brief Runs the parser.
  /// @return 0 on success.
  int parse (std::string const & f);

  /// @brief Error handling function for the scanner.
  void error (pparser::location const & l, std::string const & m);
  /// @brief Error handling function for the scanner.
  void error (std::string const & m);
  /// @brief Semantic error handling for the driver.
  [[noreturn]] void semantic_error(ast::LocationType const & l,
                                   std::string const & m) const;

  // Data structures accessed by the parser.
  std::string file;
  std::vector<ast::Label> labels;
  std::vector<ast::Constant> constants;
  std::vector<ast::Formula> formulae;
  std::vector<ast::Module> modules;
  std::vector<ast::ModuleRenaming> renamings;
  std::vector<ast::RewardSection> rewards;
  std::vector<ast::Player> players;
  std::vector<ast::Variable> globals;
  Verbosity::Level verbosity;
  ast::ModelType modelType;
private:
  /// Maps int constant names to their values.
  std::map<std::string, int> intConstantMap;
  /// Maps double constant names to their values.
  std::map<std::string, double> doubleConstantMap;
  /// Maps double constant names to their values.
  std::map<std::string, bool> boolConstantMap;

  /// Collects pairs of module index and command index.
  using CommandList = std::vector<std::pair<int,size_t>>;
  /// Maps action names to (module_index,command_index) pairs.
  using EnabledMap = std::multimap<Action,std::pair<size_t,size_t>>;
  /// Collects set of simultaneously enabled commands.
  using EnabledCommands = std::vector<std::pair<Action,CommandList>>;
  /// Maps action names to action numbers.
  using ActionMap = std::map<std::string,Action>;
  /// Maps action names to sets of modules where they are used.
  using ActionModules = std::map<Action,std::set<int>>;
  /// Counts the modules that use each named action.
  using ActionCounts = std::map<Action,int>;
  /// Maps variable names to varInfo index.
  using VarMap = std::map<std::string,size_t>;
  /// Maps formula names to formula numbers.
  using FormulaMap = std::map<std::string,size_t>;
  /// Maps variable names to values.
  using Environment = std::vector<int>;
  /// Maps module names to module number.
  using ModuleMap = std::map<std::string,size_t>;
  /// Maps names to be replaced to names replacing them.
  using ReplacementMap = std::map<std::string,std::string>;
  /// Maps variable indices to (next) values.
  using VarValueStack = std::vector<std::pair<size_t,int>>;
  /// Maps player names to player indices.
  using PlayerMap = std::map<std::string,Player>;
  /// Maps action names to player names.
  using ActionPlayer = std::map<std::string,std::string>;
  /// Maps module names to player names.
  using ModulePlayer = std::map<std::string,std::string>;
  /// Maps action index to player index.
  using ActionOwner = std::map<Action,Player>;

  /// Collects information about module variables.
  struct VarInfo {
    ast::DataType type;
    std::string name;
    int low;
    int high;
    int initial;
    int module; // -2 = unknown yet, -1 = global
  };

  Model & model;
  bool trace_scanning;
  bool trace_parsing;
  std::vector<std::pair<std::string,std::string>> const & defines;
  std::vector<VarInfo> varInfo;
  VarMap varMap;
  ActionMap actionMap;
  ActionModules actionModules;
  ActionCounts actionCounts;
  Action actnum;
  FormulaMap formulaMap;
  ModuleMap moduleMap;
  PlayerMap playerMap;
  ActionPlayer actionPlayer;
  ModulePlayer modulePlayer;
  ActionOwner actionOwner;
  size_t nameLengthLowerBound;

  /// Initializes maps for model generation.
  void buildMaps();

  /// Collects low, high and initial value of a variable.
  void getVarLimits(ast::Variable const & variable, int modIndex);

  /// Computes lower bound on length of node names.
  void setNameLengthBound();

  /// Applies command-line definitions.
  void applyDefines();

  /// Handles formula definitions.
  void substituteFormulae();

  /// Performs formula substitution in an expression.
  void subst(ast::Expression & ex);

  /// Handles module renaming.
  void renameModules();

  /// Returns a renamed copy of an expression.
  ast::Expression rename(ast::Expression const & ex,
                         ReplacementMap const & rmap) const;

  /// Folds constants.
  void foldConstants();

  /// Folds constants in an expression.
  bool foldInExpression(ast::Expression & ex);

  /// Folds constants in a primed expression.
  void foldInPrimedExpression(ast::Expression & ex);

  /// Writes the AST in PRISM format for debugging.
  void dumpAst() const;

  /// Semantic checking of the AST.  Also fixes variables types in expressions.
  void checkAst();

  /// Checks semantic restrictions on Boolean expression.
  void checkBooleanExpression(ast::Expression & ex);

  /// @brief Checks semantic restrictions on primed Boolean expressions.
  /// @details Primed Boolean expression are currently restricted to atoms
  /// of the form primed_variable = integer_expression, their conjunctions,
  /// and the special form "true."
  void checkPrimedBooleanExpression(ast::Expression & ex,
                                    int moduleIndex,
                                    bool anonymous);

  /// Checks semantic restrictions on integer expressions.
  void checkIntegerExpression(ast::Expression & ex);

  /// Checks semantic restrictions on double expressions.
  void checkDoubleExpression(ast::Expression & ex);

  /// Builds the model object from the AST.
  void buildModel();

  /// Computes intial state and counts decision states.
  Node initialState();

  /// Collects commands enabled in one state. 
  void collectEnabledCommands(Environment const & env, EnabledCommands & enabled);

  /// Builds transitions that are enabled in a state of a multimodule model.
  void buildTransitions(
    Node state,
    Environment const & parts,
    CommandList::const_iterator const & lb,
    CommandList::const_iterator const & ub,
    Action action,
    std::queue<Node> & q,
    Node & probnum) const;

  /// Recursive helper for buildTransitions.
  void buildTransitionsRecur(
    Node state,
    Node source,
    Environment const & parts,
    CommandList::const_iterator const & lb,
    CommandList::const_iterator const & ub,
    Action action,
    Probability probability,
    std::queue<Node> & q,
    VarValueStack & vstack) const;

  /// Updates the destination value based on an enabled transition.
  void updateDestination(ast::Expression const & ex,
                         VarValueStack & vstack,
                         Environment const & env) const;

  /// Gets (possibly new) action number from action name.
  Action getActionNumber(std::string const & name);

  /// Adds reward to either state or command.
  void addReward(Node state, ast::RewardSpec const & reward,
                 Environment const & env);

  /// Makes name for node.
  std::string makeName(Environment const & parts) const;

  /// Converts a vector of module variable values to a global state.
  Node encode(Environment const & parts) const;

  /// Converts a global state to a vector of module variable values.
  Environment decode(Node x) const;

  /// Increments a tuple of module variable values.
  //  Caveat emptor: currently unused and never really tested.
  void increment(Environment & p) const;

  /// Checks truth of formula for given values of free variables.
  bool isEnabled(ast::Expression const & ex,
                 Environment const & env = {}) const;

  /// Evaluates integer expression for given values of free variables.
  int evaluateIntegerExpression(ast::Expression const & ex,
                                Environment const & env = {}) const;

  /// Evaluates double expression for given values of free variables.
  double evaluateDoubleExpression(ast::Expression const & ex,
                                  Environment const & env = {}) const;

};

#endif
