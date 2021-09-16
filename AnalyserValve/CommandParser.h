#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

class CommandParser {
  public:
    CommandParser(void (*mode)(int), void (*set)(int, int), void (*error)(const char *));
    void parseInput(char);

  private:
    void doCommand();
  
    char command[256] = "";
    int position = 0;

    void (*modeFunction)(int);
    void (*setFunction)(int, int);
    void (*errorFunction)(const char *);
};

#endif // COMMAND_PARSER_H
