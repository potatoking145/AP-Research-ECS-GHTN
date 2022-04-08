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
		distance += worldstateDistance(elem.second, rhs->at(elem.first));
	}
	return distance;
}

bool Task::checkRequirments(world_t* in)
{
	for (auto elem : _actions)
	{
		if (not elem->checkRequirments(in))
			return false;
	}
	return true;
}

world_t Task::applyResult(world_t in)
{
	for (auto elem : _actions)
	{
		elem->applyResult(in);
	}
	return in;
}

AStar::AStar(uint8_t maxDepth, bool allowPlanAfterDepth)
	: _maxDepth(maxDepth),
	_allowPlanAfterDepth(allowPlanAfterDepth)
{
	_tmpWorlds.reserve(_maxDepth);
	_tmpNodes.reserve(_maxDepth);
	_openNodes.reserve(_maxDepth);
};

void AStar::setDepth(uint8_t value)
{
	_maxDepth = value;
	
	_tmpWorlds.clear();
	_tmpNodes.clear();
	_openNodes.clear();
	_tmpNodes.shrink_to_fit();
	_tmpWorlds.shrink_to_fit();
	_openNodes.shrink_to_fit();
	_tmpWorlds.reserve(_maxDepth);
	_tmpNodes.reserve(_maxDepth);
	_openNodes.reserve(_maxDepth);
}

void AStar::populatePlan(plan_t* out, aStarNode* start)
{
	auto crrntNode = start;
	while (crrntNode->g != 0)
	{
		out->push_back(crrntNode->taskTaken);
		crrntNode = crrntNode->previous;
	}
}

bool AStar::constructPlan(plan_t* out, world_t* start, world_t* goal, std::vector<Task*>* tasks)
{
	aStarNode* crrntNode = _openNodes.emplace_back(
		&_tmpNodes.emplace_back(
			&_tmpWorlds.emplace_back(*start),
			nullptr,
			nullptr,
			0,
			worldDistance(start, goal) //start node
		)
	);
	
	auto value = _tmpWorlds.max_size();
	
	bool depthReached { false };
	bool pathFound { false };
	while (not _openNodes.empty()) [[likely]]
	{
		std::sort(_openNodes.begin(), _openNodes.end(), aStarNodeCompare());
		crrntNode = _openNodes.front();
		
		if (worldEquality(goal, crrntNode->world)) [[unlikely]] {
			pathFound = true;
			break;
		} else [[likely]] { //not at the end yet
			_openNodes.erase(std::find(_openNodes.begin(), _openNodes.end(), crrntNode));
			
			for (auto neighbor : *tasks) // go through neighbors
			{
				if (neighbor->checkRequirments(crrntNode->world)) {
					world_t* newWorld = &_tmpWorlds.emplace_back(neighbor->applyResult(*crrntNode->world));
					uint32_t newF = crrntNode->g + worldDistance(newWorld, goal) + 1;
					
					if (crrntNode->f >= newF) { // this path is better or atleast the same
						auto node = _openNodes.emplace_back(
							&_tmpNodes.emplace_back(
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
		} // worldEquality else
	} // not _openNodes.empty() 
	
cleanup:
	_tmpWorlds.clear();
	_tmpNodes.clear();
	_openNodes.clear();
	
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

aiSystem::aiSystem(flecs::world* world)
{
	_system = world->system<comp_AI>()
		.iter([](flecs::iter it, comp_AI* ai) {
			auto world = it.world();
			const world_t* globalBlackBoard = world.get<world_t>();
			AStar* GHTN = world.get_mut<AStar>();
			
			for (auto i : it) {
				if (ai[i].needPlan) [[unlikely]] {
					world_t mergedBlackBoard { ai[i].internalBlackBoard };
					mergedBlackBoard.insert(globalBlackBoard->begin(), globalBlackBoard->end());
					
					ai[i].plan.clear();
					GHTN->constructPlan(&ai[i].plan, &mergedBlackBoard, &ai[i].goal, &ai[i].tasks);
					ai[i].needPlan = false;
				} else [[likely]] {
					auto* plan = &ai[i].plan;
					if (ai[i].planStep + 1 == plan->size()) [[likely]] {
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

class action1 : public IAction
{
	void applyResult(world_t& in) { in[0] = false; };
};

class action2 : public IAction
{
	void applyResult(world_t& in) { in[1]  = 20; };
};

class action3 : public IAction
{
	void applyResult(world_t& in) { in[2] = "a"; };
};
 
int main()
{
	world_t world{ {0, true}, {1, 30}, {2, "b"} };
	world_t goal{ {0, false}, {1, 20}, {2, "a"} };
	
	action1 action_1;
	action2 action_2;
	action3 action_3;
	
	Task task_1 { "t1" };
	Task task_2 { "t2" };
	Task task_3 { "t3" };
	
	task_1.addAction(&action_1);
	task_2.addAction(&action_2);
	task_3.addAction(&action_3);
	
	std::vector<Task*> tasks;
	tasks.push_back(&task_1);
	tasks.push_back(&task_2);
	tasks.push_back(&task_3);
	
	flecs::world ecs;
	
	ecs.set<AStar>( { 20, true } );
	ecs.set<world_t>( {} ); // globalBlackBoard
	
	// add a bunch of entities
	for (int i = 0; i != 10; i++) {
		ecs.entity().set<comp_AI>( {tasks, world, goal} );
	}
	
	// attach systems
	aiSystem hold { &ecs };
	
	auto start = std::chrono::high_resolution_clock::now();
	for (int i = 0; i != 1; i++) {
		ecs.progress(0);
	}
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
	
	std::wcout << "Time(ms): " << duration.count();
	
	return 0;
}
