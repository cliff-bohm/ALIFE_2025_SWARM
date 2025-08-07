//  MABE is a product of The Hintze Lab @ MSU
//     for general research information:
//         http://hintzelab.msu.edu/
//     for MABE documentation:
//         github.com/Hintzelab/MABE/wiki
//
//  Copyright (c) 2019 Michigan State University. All rights reserved.
//     to view the full license, visit:
//          github.com/Hintzelab/MABE/wiki

//  This file was auto-generated from cmake

#include <module_factories.h>

//create an archivist
auto makeArchivist(std::vector<std::string> popFileColumns, std::shared_ptr<Abstract_MTree> _maxFormula, std::shared_ptr<ParametersTable> PT, std::string groupPrefix = "") -> std::shared_ptr<DefaultArchivist> {
  std::shared_ptr<DefaultArchivist> newArchivist;
  bool found = false;
  std::string archivistType = DefaultArchivist::Arch_outputMethodStrPL->get(PT);
  if (archivistType == "Default") {
    newArchivist = std::make_shared<DefaultArchivist>(popFileColumns, _maxFormula, PT, groupPrefix);
    found = true;
    }
  if (!found){
    std::cout << "  ERROR! could not find ARCHIVIST-outputMethod \"" << archivistType << "\".\n  Exiting." << std::endl;
    exit(1);
    }
  return newArchivist;
}

//create a template brain
auto makeTemplateBrain(int inputs, int outputs, std::shared_ptr<ParametersTable> PT) -> std::shared_ptr<AbstractBrain> {
  std::shared_ptr<AbstractBrain> newBrain;
  bool found = false;
  std::string brainType = AbstractBrain::brainTypeStrPL->get(PT);
  if (brainType == "Human") {
    newBrain = HumanBrain_brainFactory(inputs, outputs, PT);
    found = true;
    }
  if (brainType == "Markov") {
    newBrain = MarkovBrain_brainFactory(inputs, outputs, PT);
    found = true;
    }
  if (!found){
    std::cout << "  ERROR! could not find BRAIN-brainType \"" << brainType << "\".\n  Exiting." << std::endl;
    exit(1);
    }
  return newBrain;
}

//create a template genome
auto makeTemplateGenome(std::shared_ptr<ParametersTable> PT) -> std::shared_ptr<AbstractGenome> {
  std::shared_ptr<AbstractGenome> newGenome;
  bool found = false;
  std::string genomeType = AbstractGenome::genomeTypeStrPL->get(PT);
if (genomeType == "Circular") {
  newGenome = CircularGenome_genomeFactory(PT);
  found = true;
  }
if (found == false){
  std::cout << "  ERROR! could not find GENOME-genomeType \"" << genomeType << "\".\n  Exiting." << std::endl;
  exit(1);
  }
return newGenome;
}

//create an optimizer
auto makeOptimizer(std::shared_ptr<ParametersTable> PT) -> std::shared_ptr<AbstractOptimizer> {
  std::shared_ptr<AbstractOptimizer> newOptimizer;
  bool found = false;
  std::string optimizerType = AbstractOptimizer::Optimizer_MethodStrPL->get(PT);
  if (optimizerType == "Tournament") {
    newOptimizer = std::make_shared<TournamentOptimizer>(PT);
    found = true;
    }
  if (!found){
    std::cout << "  ERROR! could not find OPTIMIZER-optimizer \"" << optimizerType << "\".\n  Exiting." << std::endl;
    exit(1);
    }
  return newOptimizer;
}

//create a world
auto makeWorld(std::shared_ptr<ParametersTable> PT) -> std::shared_ptr<AbstractWorld> {
  std::shared_ptr<AbstractWorld> newWorld;
  bool found = false;
  std::string worldType = AbstractWorld::worldTypePL->get(PT);
  if (worldType == "BlockCatch") {
    newWorld = std::make_shared<BlockCatchWorld>(PT);
    found = true;
    }
  if (worldType == "NBack") {
    newWorld = std::make_shared<NBackWorld>(PT);
    found = true;
    }
  if (worldType == "PathFollow") {
    newWorld = std::make_shared<PathFollowWorld>(PT);
    found = true;
    }
  if (worldType == "maxNav") {
    newWorld = std::make_shared<maxNavWorld>(PT);
    found = true;
    }
  if (worldType == "slimBerry") {
    newWorld = std::make_shared<slimBerryWorld>(PT);
    found = true;
    }
  if (!found){
    std::cout << "  ERROR! could not find WORLD-worldType \"" << worldType << "\".\n  Exiting." << std::endl;
    exit(1);
    }
  return newWorld;
}

//configure Defaults and Documentation
void configureDefaultsAndDocumentation(){
  Parameters::root->setParameter("BRAIN-brainType", (std::string)"Markov");
  Parameters::root->setDocumentation("BRAIN-brainType", "brain to be used, [Human, Markov]");
  Parameters::root->setParameter("GENOME-genomeType", (std::string)"Circular");
  Parameters::root->setDocumentation("GENOME-genomeType", "genome to be used, [Circular]");
  Parameters::root->setParameter("ARCHIVIST-outputMethod", (std::string)"Default");
  Parameters::root->setDocumentation("ARCHIVIST-outputMethod", "output method, [Default]");
  Parameters::root->setParameter("OPTIMIZER-optimizer", (std::string)"Tournament");
  Parameters::root->setDocumentation("OPTIMIZER-optimizer", "optimizer to be used, [Tournament]");
  Parameters::root->setParameter("WORLD-worldType", (std::string)"maxNav");
  Parameters::root->setDocumentation("WORLD-worldType","world to be used, [BlockCatch, NBack, PathFollow, maxNav, slimBerry]");
}
