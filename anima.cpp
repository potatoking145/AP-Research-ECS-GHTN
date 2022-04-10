#include "anima.h"

#include <iostream>
#include <chrono>

using namespace anima;

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
static int worldstateDistance(worldstate_t& lhs, worldstate_t& rhs)
{
	return std::visit(overloaded 
				{  
					[](bool lhs, bool rhs) {  return lhs == rhs ? 0 : 1; },
					[](int lhs, int rhs) {  return std::abs( rhs - lhs ); },
					[](std::string& lhs, std::string& rhs) { return lhs == rhs ? 0 : 1; },
					
					//ignored
					[](bool, int) {  return 0; },
					[](bool, std::string&) {  return 0; },
					[](int, bool) {  return 0; },
					[](int, std::string&) {  return 0; },
					[](std::string&, bool) {  return 0; },
					[](std::string&, int) {  return 0; },
				},
				lhs, rhs);
}

static bool worldEquality(world_t* lhs, world_t* rhs)
{
	for (auto elem : *lhs)
	{
		if (rhs->contains(elem.first)) {
			if (rhs->operator[](elem.first) != elem.second)
				return false;
		}
	}
	return true;
}

static int worldDistance(world_t* lhs, world_t* rhs)
{
	int distance { 0 };
	for (auto elem : *lhs)
	{
		if (rhs->contains(elem.first))
			distance += worldstateDistance(elem.second, rhs->operator[](elem.first));
	}
	return distance;
}

bool Task::checkRequirments(world_t* in)
{
	for (auto elem : _actions)
	{
		if (not elem.checkRequirments(in))
			return false;
	}
	return true;
}

world_t Task::applyResult(world_t in)
{
	for (auto elem : _actions)
	{
		elem.applyResult(in);
	}
	return in;
}

void AStar::populatePlan(plan_t* out, aStarNode* start) const
{
	auto crrntNode = start;
	while (crrntNode->g != 0)
	{
		out->push_back(crrntNode->taskTaken);
		crrntNode = crrntNode->previous;
	}
}

bool AStar::constructPlan(plan_t* out, world_t* start, world_t* goal, std::vector<Task*>* tasks) const
{
	assert(not tasks->empty());
	assert(not goal->empty());
	assert(out->empty());
	assert(not (goal->size() > start->size()));
	
	std::vector<aStarNode> tmpNodes;
	std::vector<aStarNode*> openNodes;
	
	aStarNode* crrntNode = openNodes.emplace_back(
		&tmpNodes.emplace_back( //start node
			*start,
			nullptr,
			nullptr,
			0,
			worldDistance(start, goal)
		)
	);
	
	bool depthReached { false };
	bool pathFound { false };
	while (not openNodes.empty()) [[likely]]
	{
		std::sort(openNodes.begin(), openNodes.end(), aStarNodeCompare());
		crrntNode = openNodes.front();
		
		if (worldEquality(goal, &crrntNode->world)) [[unlikely]] {
			pathFound = true;
			break;
		} else [[likely]] { //not at the end yet
			for (auto neighbor : *tasks) // go through neighbors
			{
				if (neighbor->checkRequirments(&crrntNode->world)) {
					world_t newWorld = neighbor->applyResult(crrntNode->world);
					
					uint32_t newF = crrntNode->g + worldDistance(&newWorld, goal) + 1;
					
					if (crrntNode->f >= newF) { // this path is better or atleast the same
						auto node = openNodes.emplace_back(
							&tmpNodes.emplace_back(
								newWorld,
								crrntNode,
								neighbor,
								crrntNode->g + 1,
								newF
							)
						);
						
						if (node->g >= _maxDepth) {
							depthReached = true;
							goto cleanup;
						}
					}
				} // is neighbor valid
			} // neighbors
			
			openNodes.erase(std::find(openNodes.begin(), openNodes.end(), crrntNode));
			
		} // worldEquality else
	} // not _openNodes.empty() 
	
cleanup:
	if (pathFound) {
		if (depthReached == true) {
			if (_allowPlanAfterDepth == true)
				populatePlan(out, crrntNode);
		} else {
			populatePlan(out, crrntNode);
		}
		return true;
	} else { 
		return false;
	}
}

[[nodiscard("Unable to access system later.")]] flecs::system<comp_AI> anima::attatchAISystem(flecs::world* world)
{
	return world->system<comp_AI>()
		.iter([](flecs::iter it, comp_AI* ai) {
			auto world = it.world();
			const world_t* globalBlackBoard = world.get<world_t>();
			const AStar* GHTN = world.get<AStar>();
			
			for (auto i : it) {
				if (ai[i].needPlan) [[unlikely]] {
					world_t mergedBlackBoard { ai[i].internalBlackBoard };
					mergedBlackBoard.insert(globalBlackBoard->begin(), globalBlackBoard->end());
					
					ai[i].plan.clear();
					GHTN->constructPlan(&ai[i].plan, &mergedBlackBoard, &ai[i].goal, &ai[i].tasks);
					ai[i].needPlan = false;
				} else [[likely]] {
					auto* plan = &ai[i].plan;
					if ((unsigned int)(ai[i].planStep + 1) == plan->size()) [[likely]] {
						auto task = plan->at(ai[i].planStep);
						if (task->isTaskFinished(ai[i].taskStep)) [[likely]] {
							if (not task->executeAction(ai[i].taskStep)) {
								//TODO: add logging functionaly and log failed action
								
								ai[i].planStep = 0;
								ai[i].taskStep = 0;
								ai[i].needPlan = true;
							}
							ai[i].taskStep += 1;
						} else [[unlikely]] {
							ai[i].planStep += 1;
							ai[i].taskStep = 0;
						}
					} else [[unlikely]] {
						ai[i].planStep = 0;
						ai[i].taskStep = 0;
						ai[i].needPlan = true;
					}
				}
			}
		});
}
