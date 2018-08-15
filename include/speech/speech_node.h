#ifndef SPEECH_NODE_H_
#define SPEECH_NODE_H_
/* Speech Node. Either speaks the given text or plays a given wav file. Publishes message once taling finished
 */
#include <string>
#include <ros/ros.h>
#include <speech/voice.h>
#include <dynamic_reconfigure/server.h>
#include <speech/SpeechConfig.h>

class SpeechNode
{
public:
    SpeechNode();
    
    // This callback is for when the dynamic configuration parameters change
    void reconfCallback(speech::SpeechConfig &config, uint32_t level);
    // Check if we should delay before sending message on talking_finished topic
    void speakingFinished(); 

private:
    ros::NodeHandle n_;
    ros::Subscriber voice_sub_;    
    ros::Publisher talking_finished_pub_;
    
    std::string language_;
    int pitch_;
    int bass_;
    int treble_;
    double vol_;
    bool norm_;
    
    bool finshed_speaking_;
    int loop_count_down_;

    // This callback is for when a voice message received
    void voiceCallback(const speech::voice& voice);     
};

#endif // SPEECH_NODE_H_

