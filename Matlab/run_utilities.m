addpath(genpath('utilities'));

%% Write inputs to .txt file
utilitiesObj = Utilities(config,log);
utilitiesObj.writeInputsToTxt('inputs.txt');

%% Write inputs derivatives to .txt file
utilitiesObj = Utilities(config,log);
utilitiesObj.writeInputsDerivativesToTxt('inputsDerivatives.txt');