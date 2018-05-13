#ifndef LIBPCC_CMDPARSER_HPP
#define LIBPCC_CMDPARSER_HPP

#include <string>
#include <vector>
#include <map>

/**
 * Convenience class to define command line argument interface
 * of a cmd program.
 * Can be used to set options and extract them from a command line call.
*/
class CMDParser{
 public:
  CMDParser(std::string arguments);
  ~CMDParser();

  void addOpt(std::string opt, int numValues, std::string optlong, std::string help = "");
  void showHelp();
  void init(int& argc, char** argv);

  int isOptSet(std::string opt);

  std::vector<int> getOptsInt(std::string);
  std::vector<float> getOptsFloat(std::string);
  std::vector<std::string> getOptsString(std::string);


  std::vector<std::string> getArgs() const;

 private:
  std::map<std::string,std::vector<std::string>* > _opts;
  std::map<std::string,int> _optsNumValues;
  std::map<std::string,std::string> _optslong;
  std::vector<std::string> _args;
  std::string _help;
  std::string _arguments;
};




#endif // LIBPCC_CMDPARSER_HPP
