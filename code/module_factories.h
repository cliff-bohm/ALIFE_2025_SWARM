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

#pragma once
#include <Archivist/DefaultArchivist.h>
#include <Brain/HumanBrain/HumanBrain.h>
#include <Brain/MarkovBrain/MarkovBrain.h>
#include <Genome/CircularGenome/CircularGenome.h>
#include <Optimizer/TournamentOptimizer/TournamentOptimizer.h>
#include <World/BlockCatchWorld/BlockCatchWorld.h>
#include <World/NBackWorld/NBackWorld.h>
#include <World/PathFollowWorld/PathFollowWorld.h>
#include <World/maxNavWorld/maxNavWorld.h>
#include <World/slimBerryWorld/slimBerryWorld.h>

//create-archivist factory signature
auto makeArchivist(std::vector<std::string> /*popFileColumns*/, std::shared_ptr<Abstract_MTree> _maxFormula, std::shared_ptr<ParametersTable> PT, std::string /*groupPrefix*/) -> std::shared_ptr<DefaultArchivist>;

//create-template-brain factory signature
auto makeTemplateBrain(int /*inputs*/, int /*outputs*/, std::shared_ptr<ParametersTable> /*PT*/) -> std::shared_ptr<AbstractBrain>;

//create-template-genome factory signature
auto makeTemplateGenome(std::shared_ptr<ParametersTable> /*PT*/) -> std::shared_ptr<AbstractGenome>;

//create an optimizer
auto makeOptimizer(std::shared_ptr<ParametersTable> /*PT*/) -> std::shared_ptr<AbstractOptimizer>;

//create-world factory signature
auto makeWorld(std::shared_ptr<ParametersTable> PT) -> std::shared_ptr<AbstractWorld>;

//Defaults and Documentation signature
auto configureDefaultsAndDocumentation() -> void;
