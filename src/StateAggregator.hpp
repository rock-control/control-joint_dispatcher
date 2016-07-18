#ifndef STATEAGGREGATOR_HPP
#define STATEAGGREGATOR_HPP
#include <vector>
#include <map>
#include <base/Time.hpp>
#include <base/commands/Joints.hpp>
#include <boost/function.hpp>

namespace joint_dispatcher
{

class StateAggregator
{
    class StateInfo
    {
    public:
        size_t stateId;
        size_t outputPosition;
        bool updated;
        base::Time updateTime;
    };
    
    ///amount of dispatched actuators
    size_t actuatorCnt;
    
    ///conter for arrived state updates
    size_t updateCnt;
    
    base::commands::Joints status;
    std::vector<StateInfo *> stateIdToInfo;
    
    std::map<size_t, StateInfo> stateMap;

    ///The nominal time between two state updates
    base::Time statusInterval;
    
    boost::function<void (const base::commands::Joints &status)> writeStatusCallback;
    
    bool writeState;
protected:
    const base::commands::Joints &getStatus() const 
    {
        return status;
    };
    
public:
    
    const std::vector<size_t> getActuatorIds() const;
    
    StateAggregator(const std::vector< std::pair<size_t, std::string> >& stateMap, boost::function<void (const base::commands::Joints &status)> writeStatusCallback, base::Time statusInterval);
    ~StateAggregator() {};
    
    /**
     * Updates a single status in the aggregator.
     * This function also triggers the write of teh full state.
     * */
    void setNewStatus(size_t stateId, const base::Time stateTime, const base::JointState& state);

    /**
     * Checks if we got a full state and writes it
     * using the writeStatusCallback function.
     * Should be called regulary
     * */
    void checkWriteState();
    
    
    /**
    * Resets the aggregator state while keeping
    * the id mapping and the names in the state.
    * */
    void reset();
};

class CommandDispatcher
{
    class InputMapping
    {
    public:
        int actuatorId;
        int inputId;
    };    
    
    ///map from positions in the base::actuator::command to the driver ids
    std::vector<InputMapping> inputMap;
    
    boost::function<void (int32_t actuatorId, base::JointState::MODE mode, double value)> setCommandCallback;
    
public:
    const std::vector<int32_t> getActuatorIds() const;
    
    CommandDispatcher(std::vector< boost::int32_t > const & actuatorMap, boost::function<void (int32_t actuatorId, base::JointState::MODE mode, double value)> setCommandCallback);
    virtual ~CommandDispatcher() {};
    
    void processCommand(base::commands::Joints cmd);    
};

}

#endif // STATEAGGREGATOR_HPP
