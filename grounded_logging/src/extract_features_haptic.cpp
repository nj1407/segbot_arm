#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>  
#include <fstream>
#include <iostream>
// For traversing the filesystem
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp> 

using namespace std;
using namespace boost::filesystem;

// The general file path
std::string generalFilePath = "/home/users/pkhante/grounded_learning_experiments/";

// File path to save data
string saveFilePath = "/home/users/pkhante/extracted_feature_vectors/";

//Name of the audio folder to search for
const std::string & haptic_folder = "haptic_data";

//Total number of objects and trials for ease of file traversal
int total_objects = 32, total_trials = 6;

// Create a vector to put in the dft datapoints
vector<vector<double> > haptic_data;

int timeBins = 10;
int effortBins = 6;

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "extract_features_haptic");
	ros::NodeHandle nh;
	ROS_INFO("Ready to extract features from the haptic files...");
	
	for(int object_num = 1; object_num <= total_objects; object_num++){
		for(int trial_num = 1; trial_num <= total_trials; trial_num++){
			std::stringstream convert1, convert2;
			convert1 << object_num;
			convert2 << trial_num;
			string filePath = generalFilePath + "obj_" + convert1.str() + "/trial_" + convert2.str();
			path dir_path(filePath);
			if(!exists(dir_path))
				return 1;
				
			directory_iterator end_itr;       // default construction yields past-the-end
			for(directory_iterator itr(dir_path); itr != end_itr; ++itr){
				string folder_name = itr->path().filename().string();
				directory_iterator end_itr2;       // default construction yields past-the-end
				for(directory_iterator itr2(itr->path()); itr2 != end_itr2; ++itr2){
					if(itr2->path().filename().string() == haptic_folder){
						//Go through all the files and find the haptic files
						directory_iterator itr3(itr2->path()), eod;
						BOOST_FOREACH(path const &p, std::make_pair(itr3, eod)){ 
							if(is_regular_file(p)){
								// Get the dft.txt file
								if(p.extension() == ".csv"){
									//cout << p.stem();
									string filePath = itr2->path().string() + "/" + p.filename().string();
									ROS_INFO("Filepath: %s", filePath.c_str());
									
									// Read in the file
									ifstream infile(filePath.c_str());
									string line;
									int num_of_lines = 0; 
									
									// Count the number of rows in the file as the columns stay 65 always
									for(int i=0; getline(infile, line); ++i){
										num_of_lines++;
									}
									
									//ROS_INFO("Number of lines: %d", num_of_lines);
																		
									ifstream infile2(filePath.c_str());
									if (infile2.is_open())
									{
										string line;
										while(getline(infile2, line)){
											typedef boost::escaped_list_separator<char> Separator;
											typedef boost::tokenizer<Separator> Tokenizer;
											
											// Skip the headings of the csv file
											if(line.find("efforts_q1") != string::npos)
												continue;
											
											// Fill the haptic_data matrix
											Tokenizer tokenizer(line);
											int count = 0;
											vector<double> haptic_row;
											for (Tokenizer::iterator iter = tokenizer.begin(); (iter != tokenizer.end()) && (count < 6); ++iter)
											{
												if ((count >= 0) & (count <=5))
												{
													stringstream sstr(*iter);
													double value;
													while(sstr >> value){
														haptic_row.push_back(value);
													}
												}
												count++;
											}
											haptic_data.push_back(haptic_row);
										}
									}
									
									// Create a vector to store the downsampled 10X6 matrix
									double  histData[10][6];
									int tBinLength = (int)floor((double)(num_of_lines-1)/(double)timeBins);
									int fBinLength = (int)floor((double)haptic_data[0].size()/(double)effortBins);
									
									//ROS_INFO("Fwidth: %d", fBinLength);
									//ROS_INFO("Twidth: %d", tBinLength);
									
									for (int fIndex = 0; fIndex < effortBins; fIndex++){
										for (int tIndex = 0; tIndex < timeBins; tIndex++){
											int tStart = tIndex*tBinLength;
											int tEnd = min((tIndex+1)*tBinLength, num_of_lines-1);
											
											int fStart = fIndex*fBinLength;
											int fEnd = min((fIndex+1)*(fBinLength), (int)haptic_data[0].size());
										
											int c = 0;
											double value = 0;
											for (int i = tStart; i < tEnd; i ++){
												for (int j = fStart; j < fEnd; j ++){
													value += haptic_data[i][j];
													c++;
												}
											}
											value = value/(double)c;
											
											//ROS_INFO("Index: %d  %d   %d", fIndex, fStart, fEnd);
											//ROS_INFO("INdex: %d  %d   %d", tIndex, tStart, tEnd);
											
											histData[tIndex][fIndex] = value;
										}
									}
									
									// Print out the histogram matrix
									/*for(int x=0; x<10; x++){
										for (int y=0; y<10; y++){
											cout<<histData[x][y];
										}
										cout<<endl<<endl<<endl;
									}*/
									
									// Write the historgram matrix to a file
									string file_to_write = saveFilePath+"/"+folder_name+"_haptics"+"/haptic_extracted_features.csv";
									//ROS_INFO("File to write path: %s", file_to_write.c_str());
									ofstream haptic_file(file_to_write.c_str(), std::ios::out | std::ios::app);
									ROS_INFO("Writing to the file...");
									if(haptic_file.is_open()){
										for(int x=0; x<10; x++){
											haptic_file << "test"+convert1.str()+"_trial"+convert2.str();
											haptic_file << ",";
											for (int y=0; y<6; y++){
												haptic_file << histData[x][y];
												if(y==5)
													haptic_file << "\n";
												else
													haptic_file << ",";
											}
										}
										haptic_file.close();
										ROS_INFO("File saved");
									}
									else 
										ROS_ERROR("Cannot open file");
									
									// Clear out the haptic_data
									haptic_data.clear();
								}	
							}
						}
					}
				}
			}
		}
	}
	return 0;
}

