#include "CommandParser.h"

#include <stdio.h>
#include <ctype.h>

CommandParser::CommandParser(void (*info)(int), void (*mode)(int), void (*getter)(int), void (*setter)(int, int), void (*error)(const char *)) {
  infoFunction = info;
  modeFunction = mode;
  getFunction = getter;
  setFunction = setter;
  errorFunction = error;
}

void CommandParser::parseInput(char input) {
  if (input != '\n' && input != '\r') {
    command[position++] = input;
  } else {
    command[position] = 0;
    if (position != 0) {
      doCommand();

      position = 0;
    }
  }
}

void CommandParser::doCommand() {
  int index;
  int intParam;

  command[0] = toupper(command[0]);

  switch (command[0]) {
    case 'I':
      if (sscanf(command, "I%d", &index) > 0) {
        infoFunction(index);
      }
      break;
    case 'M':
      if (sscanf(command, "M%d", &index) > 0) {
        modeFunction(index);
      }
      break;
    case 'G':
      if (sscanf(command, "G%d", &index) > 0) {
        getFunction(index);
      }
      break;
    case 'S':
      intParam = -1;
      if (sscanf(command, "S%d %d", &index, &intParam) > 0) {
        setFunction(index, intParam);
      }
      break;
    default:
        errorFunction(command);
      break;
  }

}
