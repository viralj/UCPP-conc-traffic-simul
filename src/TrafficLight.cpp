#include <iostream>
#include <random>
#include "TrafficLight.h"

/* Implementation of class "MessageQueue" */

 
template <typename T>
T MessageQueue<T>::receive()
{
    // FP.5a : The method receive should use std::unique_lock<std::mutex> and _condition.wait() 
    // to wait for and receive new messages and pull them from the queue using move semantics. 
    // The received object should then be returned by the receive function. 
    
    std::unique_lock<std::mutex> uLock(_mutex);
    _cond.wait(uLock,[this]{return !_queue.empty();});

    // Message
    T msg = std::move(_queue.front());
    _queue.pop_front();

    return msg;
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex> 
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.
    std::lock_guard<std::mutex> lck(_mutex);

    // Implement as 'mail box' to prevent queue growth @ every state change.
    // Empty Queue under lock when new state received
    while (!_queue.empty()){
        _queue.pop_back();
    }
    _queue.push_back(std::move(msg));
    _cond.notify_one();
    std::cout << "MSG::send() " << _queue.size() << std::endl;
}


/* Implementation of class "TrafficLight" */
TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
}

void TrafficLight::waitForGreen()
{
    // FP.5b : add the implementation of the method waitForGreen, in which an infinite while-loop 
    // runs and repeatedly calls the receive function on the message queue. 
    // Once it receives TrafficLightPhase::green, the method returns.
    std::unique_lock<std::mutex> uLock(_mutex);
    uLock.unlock();
    while (true)
    {
        // Check for state
        auto phase = _messages.receive();
        auto lightPhase = (phase == TrafficLightPhase::red) ? "Red" : "Green";
        std::cout << "TrafficLight " << _id  << " received " << lightPhase << std::endl;

        if (phase == TrafficLightPhase::green)
            break;
    }
    
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{   
    std::lock_guard<std::mutex> lck(_mutex);
    return _currentPhase;
}

void TrafficLight::simulate()
{
    // FP.2b : Finally, the private method „cycleThroughPhases“ should be started in a thread when the public method „simulate“ is called. To do this, use the thread queue in the base class.
    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    std::unique_lock<std::mutex> uLock(_mutex);
    uLock.unlock();
    // FP.2a : Implement the function with an infinite loop that measures the time between two loop cycles 
    // and toggles the current phase of the traffic light between red and green and sends an update method 
    // to the message queue using move semantics. The cycle duration should be a random value between 4 and 6 seconds. 
    // Also, the while-loop should use std::this_thread::sleep_for to wait 1ms between two cycles. 

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(4000,6000); // ms

    double cycleDuration = dist(gen);

    // initialize stop watch  
    std::chrono::time_point<std::chrono::system_clock> lastUpdate;
    lastUpdate = std::chrono::system_clock::now();

    while (true){
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // compute time difference
        long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();
        if (timeSinceLastUpdate >= cycleDuration)
        {
            uLock.lock();
            
            if (_currentPhase == red){
                _currentPhase = green;
                std::cout << "TrafficLight " << _id << " Green." << std::endl;
            }
            else{
                _currentPhase = red; 
                std::cout << "TrafficLight " << _id << " Red." << std::endl;
            }
            // Send message to the Message Queue
            _messages.send(std::move(TrafficLightPhase(_currentPhase)));
            uLock.unlock();

            lastUpdate = std::chrono::system_clock::now();
        }

    }
}