#include "StateAggregator.hpp"
#include <iostream>
#include <math.h>

namespace joint_dispatcher
{
    
StateAggregator::StateAggregator(const std::vector< std::pair<size_t, std::string> >& actuatorMap, boost::function<void (const base::commands::Joints &status)> writeStatusCallback, base::Time statusInterval) : 
    actuatorCnt(0), updateCnt(0), statusInterval(statusInterval), writeStatusCallback(writeStatusCallback)
{
    actuatorCnt = actuatorMap.size();
    status.resize(actuatorCnt);

    for (size_t i = 0; i < actuatorCnt; ++i)
    {
        size_t board_idx = actuatorMap[i].first;
        
        StateInfo info;
        info.updated = false;
        info.stateId  = board_idx;
        info.outputPosition = i;
        status.names[i] = actuatorMap[i].second;
        
        stateMap.insert(std::make_pair(board_idx, info));
        
        if(stateIdToInfo.size() <= board_idx)
        {
            stateIdToInfo.resize(board_idx + 1, NULL);
        }
        stateIdToInfo[board_idx] = &(stateMap[board_idx]);
    }

    reset();
}

const std::vector<size_t> StateAggregator::getActuatorIds() const
{
    std::vector<size_t> ret;
    for(std::map<size_t, StateInfo>::const_iterator it = stateMap.begin(); it != stateMap.end(); it++)
    {
        ret.push_back(it->second.stateId);
    }
    return ret;
}


void StateAggregator::reset()
{
    writeState = false;
    
    //reinit the update counter
    updateCnt = actuatorCnt;
    
    //reset the updated flag
    for(std::map<size_t, StateInfo>::iterator it = stateMap.begin(); it != stateMap.end(); it++)
    {
        it->second.updated = false;
    }
}


void StateAggregator::setNewStatus(size_t stateId, const base::Time stateTime, const base::JointState& state)
{
    
    StateInfo &info(*(stateIdToInfo[stateId]));
    
    if(status.time > stateTime)
    {
        std::cout << "Warning, got state update for actuator from the past " << status.time << " " << stateTime << std::endl;
    }
    
    //if the time of the samples differ by more than an interval 
    //than the stateSet is missaligned in time.
    //if info.updated is set we lost a sample.
    //Anyway, we delay the write of the status 
    //and hope that the rest of the status will 
    //arrive before the call to checkWriteState
    if(info.updated || (stateTime - status.time > statusInterval))
    {
        writeState |= true;
    }

    if(!info.updated)
        //nominal case
        updateCnt--;
    
    info.updated = true;
    info.updateTime = stateTime;
    status[info.outputPosition] = state;

    //allways use the latest time
    status.time = std::max(stateTime, status.time);

    //if we got updates for all joints,
    //this means we have a full and valid state,
    //allways send it out directly
    if(updateCnt <= 0)
    {
        writeState = true;
        checkWriteState();
    }
}

void StateAggregator::checkWriteState()
{
    if(writeState)
    {
        writeStatusCallback(status);
        reset();
    }
}


CommandDispatcher::CommandDispatcher(const std::vector< int32_t >& actuatorMap, boost::function<void (int32_t actuatorId, base::JointState::MODE mode, double value)> setCommandCallback): setCommandCallback(setCommandCallback)
{
    for(uint32_t i = 0; i < actuatorMap.size();i++)
    {
        int board_idx = abs(actuatorMap[i]) - 1;
        //zero has the special meaning
        //of 'do not use this id'
        if(actuatorMap[i] == 0)
        {
            continue;
        }
        InputMapping m;
        m.actuatorId = board_idx;
        m.inputId = i;
        
        inputMap.push_back(m);
    }   
}

const std::vector<int32_t> CommandDispatcher::getActuatorIds() const
{
    std::vector<int32_t> ret;
    for(std::vector<InputMapping>::const_iterator it = inputMap.begin(); it != inputMap.end(); it++)
    {
        ret.push_back(it->actuatorId);
    }
    return ret;

}

void CommandDispatcher::processCommand(base::samples::Joints cmd)
{
    for(std::vector<InputMapping>::const_iterator it = inputMap.begin(); it != inputMap.end(); it++)
    {
        const int i = it->inputId;
        
        setCommandCallback(it->actuatorId, cmd[i].getMode(), cmd[i].getField(cmd[i].getMode()));
    }
};

}
