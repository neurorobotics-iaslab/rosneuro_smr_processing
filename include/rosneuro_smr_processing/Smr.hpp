#ifndef SMR_HPP
#define SMR_HPP

#include <algorithm>
#include <iostream>
#include <numeric>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "rosneuro_msgs/NeuroFrame.h"
#include "rosneuro_msgs/NeuroOutput.h"

#include "rosneuro_buffers_ringbuffer/RingBuffer.hpp"
#include "rosneuro_filters_laplacian/Laplacian.hpp"
#include "rosneuro_pwelch/Pwelch.hpp"
#include "rosneuro_decoder/Decoder.h"
#include "rosneuro_decoder_gaussian/Gaussian.h"

namespace rosneuro{
    namespace processing{

class Smr {
	public:
		Smr(void);
		virtual ~Smr(void);

		bool configure(void);
        bool classify(void);
        void run(void);
        
    private:
        void set_message(void);
		void on_received_data(const rosneuro_msgs::NeuroFrame& msg);

	private:
		ros::NodeHandle		   nh_;
		ros::NodeHandle		   p_nh_;
		ros::Subscriber		   sub_data_;
		ros::Publisher		   pub_data_;
		std::string            sub_topic_data_;
		std::string	           pub_topic_data_;
		int                    nsamples_;
		int                    nchannels_;
		rosneuro_msgs::NeuroOutput out_;

		rosneuro::Buffer<double>*   	buffer_;
		rosneuro::Filter<double>*	    laplacian_;
		rosneuro::Pwelch<double>* 	    pwelch_;
		rosneuro::decoder::Decoder*     decoder_;


	    rosneuro::DynamicMatrix<float>  data_in_;
	    rosneuro::DynamicMatrix<double> data_lap_;
	    rosneuro::DynamicMatrix<double> psd_;
        Eigen::VectorXf                 rawProb_;
        bool has_new_data_;
        bool is_first_message_;

        std::ofstream outputFile_features_;
        std::ofstream outputFile_rawprobs_;


};

Smr::Smr(void) : p_nh_("~") {
    this->sub_topic_data_ = "/neurodata";
    this->pub_topic_data_ = "/smr/neuroprediction";

    this->buffer_ = new rosneuro::RingBuffer<double>();
    this->laplacian_ = new rosneuro::Laplacian<double>();
    this->pwelch_ = new rosneuro::Pwelch<double>();
	this->decoder_ = new rosneuro::decoder::Decoder();
    this->has_new_data_ = false;
    this->is_first_message_ = true;
}

Smr::~Smr(void){
    this->outputFile_features_.close();
    this->outputFile_rawprobs_.close();
}

bool Smr::configure(void){
	
    // subscriber and advertiser nodes
	this->pub_data_ = this->p_nh_.advertise<rosneuro_msgs::NeuroOutput>(this->pub_topic_data_, 1);
    this->sub_data_ = this->nh_.subscribe(this->sub_topic_data_, 1, &Smr::on_received_data, this);

    // Configure buffer
    if(this->buffer_->configure("RingBufferCfg") == false) { // -> the buffer's parameters are saved in a yaml file
		ROS_ERROR("[%s] buffer configuration failed", this->buffer_->name().c_str());
		return false;
	}
	ROS_INFO("[%s] buffer configuration succeeded", this->buffer_->name().c_str());

    // Configure laplacian
    if(this->laplacian_->configure("LaplacianCfg") == false) { // -> the laplacian's parameters are saved in a yaml file
		ROS_ERROR("[%s] filter configuration failed", this->laplacian_->name().c_str());
		return false;
	}
	ROS_INFO("[%s] filter configuration succeeded", this->laplacian_->name().c_str());

    // Configure the pwelch
    // take parameters values
    int samplerate, wlength, wtype, novl, dolog;
    if(ros::param::get("~nchannels", this->nchannels_) == false){
        ROS_ERROR("[pwelch] Missing 'nchannels' parameter, which is a mandatory parameter");
        return false;
    }
    if(ros::param::get("~nsamples", this->nsamples_) == false){
        ROS_ERROR("[pwelch] Missing 'nsamples' parameter, which is a mandatory parameter");
        return false;
    }
    if(ros::param::get("~samplerate", samplerate) == false){
        ROS_ERROR("[pwelch] Missing 'samplerate' parameter, which is a mandatory parameter");
        return false;
    }
    if(ros::param::get("~wlength", wlength) == false){
        ROS_ERROR("[pwelch] Missing 'wlength' parameter, which is a mandatory parameter");
        return false;
    }
    if(ros::param::get("~wtype", wtype) == false){
        ROS_ERROR("[pwelch] Missing 'wtype' parameter, which is a mandatory parameter");
        return false;
    }
    if(ros::param::get("~novl", novl) == false){
        ROS_ERROR("[v] Missing 'novl' parameter, which is a mandatory parameter");
        return false;
    }
    if(ros::param::get("~dolog", dolog) == false){
        ROS_ERROR("[pwelch] Missing 'dolog' parameter, which is a mandatory parameter");
        return false;
    }
    if(this->pwelch_->configure(wlength, wtype, novl, samplerate, dolog) == false){
        ROS_ERROR("FFTW plan not created");
		return false;
    }
	ROS_INFO("[pwelch] pwelch configuration succeeded");

	// Configure the decoder
	if(!this->decoder_->configure()){
		ROS_ERROR("[decoder] error in the configuration of the decoder");
        return false;
	}

    const std::string fileoutput1 = "/home/paolo/rosneuro_ws/src/rosneuro_smr_processing/features.csv";
    const std::string fileoutput2 = "/home/paolo/rosneuro_ws/src/rosneuro_smr_processing/rawprobs.csv";
    this->outputFile_features_.open(fileoutput1);
    if(this->outputFile_features_.is_open()){
        std::cout << "file for features opened" << std::endl;
    }
    this->outputFile_rawprobs_.open(fileoutput2);
    if(this->outputFile_rawprobs_.is_open()){
        std::cout << "file for features opened" << std::endl;
    }
	
	return true;
}

void Smr::run(void){
    ros::Rate r(512);

    while(ros::ok()){

        if(this->has_new_data_ == true){
            if(!this->classify()){
                this->has_new_data_ = false;
                continue;
            }

            this->set_message();
            this->pub_data_.publish(this->out_);
            this->has_new_data_ = false;
        }

        ros::spinOnce();
        r.sleep();
    }
}

bool Smr::classify(void){
    const static Eigen::IOFormat format(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");


    this->buffer_->add(this->data_in_.transpose().cast<double>()); // [samples x channels]

    if(!this->buffer_->isfull()){
        return false;
    }

    this->data_lap_ = this->laplacian_->apply(this->buffer_->get());

    this->psd_ = this->pwelch_->apply(this->data_lap_);

    Eigen::Matrix<float, Eigen::Dynamic, 1> features = this->decoder_->getFeatures(this->psd_.transpose().cast<float>());
    this->rawProb_ = this->decoder_->apply(features);

    Eigen::Matrix<float, Eigen::Dynamic, 1> probs = this->rawProb_;

    if(this->outputFile_features_.is_open()){
        ROS_INFO("features size: %ld x %ld", features.rows(), features.cols());
        this->outputFile_features_ << features.transpose().format(format) << std::endl;
        //this->outputFile_features_ << features.format(this->format);
        
        /*
        for(int i = 0; i < features.cols(); i++){
            for(int j = 0; j < features.rows(); j++){
                //this->outputFile_features_ << features(j, i) << " ";
                std::cout << features(j,i) << " ";
            }
            //this->outputFile_features_ << std::endl;
            std::cout << std::endl;
        }
        */
    }

    if(this->outputFile_rawprobs_.is_open()){
        ROS_INFO("prob size: %ld x %ld", this->rawProb_.rows(), this->rawProb_.cols());
        this->outputFile_rawprobs_ << probs.transpose().format(format) << std::endl;
        /*
        for(int i = 0; i < this->rawProb_.cols(); i++){
            for(int j = 0; j < this->rawProb_.rows(); j++){
                //this->outputFile_rawprobs_ << this->rawProb_(j, i) << " ";
                std::cout << this->rawProb_(j,i) << " ";
            }
            //this->outputFile_rawprobs_ << std::endl;
            std::cout << std::endl;
        }
        */
    }

    return true;
}

void Smr::set_message(void){
    this->out_.header.stamp = ros::Time::now();
	this->out_.softpredict.data = std::vector<float>(this->rawProb_.data(), this->rawProb_.data() + this->rawProb_.rows() * this->rawProb_.cols());
    
}

void Smr::on_received_data(const rosneuro_msgs::NeuroFrame& msg) {

    this->has_new_data_ = true;

	// Getting pointer to the input data message
	float* ptr_in;
	ptr_in = const_cast<float*>(msg.eeg.data.data());
	this->data_in_ = Eigen::Map<rosneuro::DynamicMatrix<float>>(ptr_in, this->nchannels_, this->nsamples_); // [nchannels x nsamples]

    // set value for the message
    this->out_.neuroheader = msg.neuroheader;

    if(this->is_first_message_ == true) {
		this->out_.decoder.classes    = this->decoder_->classes();
		this->out_.decoder.type 	  = this->decoder_->name();
		this->out_.decoder.path 	  = this->decoder_->path();
		this->is_first_message_ = false;
	}
}
}
}
#endif
