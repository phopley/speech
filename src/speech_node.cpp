/* Speech Node. Either speaks the given text or plays a given wav file. Publishes message once taling finished
 */
#include <speech/speech_node.h>
#include <std_msgs/String.h>


// This callback is for when the dynamic configuration parameters change
void SpeechNode::reconfCallback(speech::SpeechConfig &config, uint32_t level)
{
    language_ = config.lang;
    vol_ = config.vol;
    pitch_ = config.pitch;
    bass_ = config.bass;
    treble_ = config.treble;
    norm_ = config.norm;
}
//---------------------------------------------------------------------------

// This callback is for when a voice message received
void SpeechNode::voiceCallback(const speech::voice& voice)
{               
    // Check to see if we have a path to a stock wav file
    if(voice.wav != "")
    {       
        // Play stock wav file
        // Use play in sox (Sound eXchange) to play the wav file, 
        // --norm stops clipping and -q quite mode (no console output)
        std::string str = "play " +  voice.wav + " --norm -q";        
                                             
        ROS_DEBUG("%s", str.c_str());               
        
        if(system(str.c_str()) != 0)
        {
            ROS_DEBUG("SpeechNode: Error on wav playback");            
        }
    }
    else
    {                        
        std::string filename = "/tmp/robot_speach.wav";                         
        
        std::string str;
        
        // create wav file using pico2wav from adjusted text.
        str = "pico2wave --wave=" + filename + " --lang=" + language_ + " \"" + voice.text + "\"";
        
        ROS_DEBUG("%s", str.c_str());
        
        if(system(str.c_str()) != 0)
        {
            ROS_DEBUG("SpeechNode: Error on wav creation");            
        }
        else
        {                
            // Play created wav file using sox play but use parameters bass, treble, pitch and vol 
            std::string bass = " bass " + std::to_string(bass_);
            std::string treble = " treble " + std::to_string(treble_);
            std::string pitch = " pitch " + std::to_string(pitch_);        
        
            if(norm_ == true)
            {            
                str = "play " +  filename + " --norm -q" + pitch + bass + treble;          
            }
            else
            {
                std::string volume = " vol " + std::to_string(vol_);
                str = "play " +  filename + " -q" + pitch + bass + treble + volume;           
            }
        
            ROS_DEBUG("%s", str.c_str());               
        
            if(system(str.c_str()) != 0)
            {
                ROS_DEBUG("SpeechNode: Error on wav playback");            
            }
        }
    }
    
    // Send talking finished 
    std_msgs::String msg;   
    msg.data = "";
    talking_finished_pub_.publish(msg);
}
//---------------------------------------------------------------------------
        
// Constructor 
SpeechNode::SpeechNode()
{
    voice_sub_ = n_.subscribe("/speech/to_speak", 5, &SpeechNode::voiceCallback, this);

    talking_finished_pub_ = n_.advertise<std_msgs::String>("/robot_face/talking_finished", 5);
}
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "speech_node");
			
	SpeechNode *speech_node = new SpeechNode();
	
	dynamic_reconfigure::Server<speech::SpeechConfig> server;
    dynamic_reconfigure::Server<speech::SpeechConfig>::CallbackType f;
  	
  	f = boost::bind(&SpeechNode::reconfCallback, speech_node, _1, _2);
    server.setCallback(f);
  
    std::string node_name = ros::this_node::getName();
	ROS_INFO("%s started", node_name.c_str());
	ros::spin();
	return 0;
}

