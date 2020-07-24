// SE_Task3.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <string> 
#include <vector>
#include <functional>   // std::bit_xor
#include <numeric>		// std::accumulator
#include <sstream>
#include <stdexcept>	// std::invalid_argument
#include <fstream>
#include "../headers/parseNMEA.h"
#include "../headers/position.h"



namespace GPS
{
        
    bool isValidSentence(const std::string &);
    NMEAPair decomposeSentence(const std::string & nmeaSentence);
    Position extractPosition( const NMEAPair &);
    std::vector<Position> routeFromNMEALog(const std::string & filepath);

    
    bool isValidSentence(const std::string & sentence)
    {
        bool valid = false;
        try{
            // Gets the first 3 letters (should = '$GP')
            std::string prefix = sentence.substr(0, 3);                
        
            // Gets the identifier ('GLL')
            std::string identifier = sentence.substr(3, 3);             

            std::size_t star = sentence.find('*');                      
            int lengthOfFields = (star - 7);
            
            // Starts at 7 to remove first comma
            std::string fields = sentence.substr(7, lengthOfFields);    

            // Although the checksum is usually the two digits after the asterisk, writing it this way makes sure that if there is invalid data at the end of the sentence, it is recognised. 
            // (Change made after testing)
            std::string checksum = sentence.substr((star + 1), (sentence.length() - star));

            bool prefixValid = false;
            bool idValid = false;
            bool checksumValid = false;

            if (prefix == "$GP")
            {
                prefixValid = true;
            }

            // Checking that a valid sentence type has been given, further types could be added in future.
            if (identifier == "GLL" || identifier == "GGA" || identifier == "RMC" )
            {
                idValid = true;
            }

            // Check that checksum == XOR(1:star-1)
            
            // Saving the part of the sentence needed for the calculation as a string.
            std::string charValues = sentence.substr(1, star-1);
            std::vector<int> intValues;
            
            // Pushing all the character values that are needed into a vector
            for (size_t i = 0; i < charValues.length(); i++)
            {
                intValues.push_back(charValues[i]);
            }

            // Using bitwise XOR to find the XOR reduction of all character values 
            int xorReduction = std::accumulate(intValues.begin(), intValues.end(), 0, std::bit_xor<int>());

            // Converting the XOR reduction from decimal to hex so it can be compared with the checksum
            // Both uppercase and lowercase hex values are found, as the input could be either
            std::stringstream ss;
            ss << std::hex << xorReduction; 
            std::string hexValueLower(ss.str());
            
            std::stringstream tt;
            tt << std::uppercase << std::hex << xorReduction;
            std::string hexValueUpper(tt.str());

            // Checking that the sentence is valid and the checksum is equal to the XOR reduction
            if ( (hexValueLower == checksum) || (hexValueUpper == checksum) ) {
                checksumValid = true;
            }
            
            // If all of the criteria has been met, then the sentence is valid
            if (prefixValid && idValid && checksumValid) {
                valid = true;
            }
        }
        
        catch (std::out_of_range) {
            //the sentence is not valid
        }



        return valid;
    }


    // Decomposes sentence into strings: the sentence type and a vector of fields
    NMEAPair decomposeSentence(const std::string & nmeaSentence) {
        NMEAPair decomposed;
        // Gets the sentence type
        std::string sentenceType = nmeaSentence.substr(1, 5);             
        decomposed.first = sentenceType;
        
        std::vector<std::string> allFields;
        std::string field;
        
        // Getting the index of the asterisk in the string, as this is where the fields stop
        std::size_t star = nmeaSentence.find('*');                      

        // adding each 
        for (size_t i = 7; i < star; i++) {
            if (nmeaSentence[i] != ',' ) {
                field += nmeaSentence[i];
            }
            else {
                allFields.push_back(field);
                field = "";
            }
        }	

        // This makes sure the last part of the sentence is pushed to the vector
        allFields.push_back(field);
        
        
        // Adding the vector of fields to the second part of the NMEA pair
        decomposed.second = allFields;

        return decomposed;
    }



    Position extractPosition(const NMEAPair & target) {
        
        std::string sentenceType = target.first;

        std::vector<std::string> fields = target.second;
        if (fields.size() == 0) {
            throw std::invalid_argument("This is not a valid sentence.");
        }

        // The following vectors hold the indexes for the GLL, GGA and RMC formats respectively
        // 'Index' is where the number of the lat/long/ele is stored
        // 'Unit' is where the direction is stored (i.e N/S E/W) and the Unit of elevation (should equal 'M' for metres)
        std::vector<int> latitudeIndex = {0, 1, 2};
        std::vector<int> latitudeChar = {1, 2, 3};
        std::vector<int> longitudeIndex = {2, 3, 4};
        std::vector<int> longitudeChar = {3, 4, 5};
        std::vector<int> elevationIndex = {0, 8, 0};
        std::vector<int> elevationUnit = {0, 9, 0};
        int position;
        
        // Further sentence types can be added here
        if (sentenceType == "GPGLL") {
            position = 0;
        }
        
        else if(sentenceType == "GPGGA") {
            position = 1;
        }
        else if(sentenceType == "GPRMC") {
            position = 2;
        }
        else {
            //This means it is not a valid sentence type
            throw std::invalid_argument("Invalid argument");
        }
        
        // The following variables find the longitude, latitude and elevation from each of the strings, taken from the index 'position' which refers to their sentence type.
        const std::string latitudeStr = fields[latitudeIndex[position]];
        const std::string longitudeStr = fields[longitudeIndex[position]];
        const std::string elevationStr = fields[elevationIndex[position]];
        
        const std::string northingStr = fields[latitudeChar[position]];
        const std::string eastingStr = fields[longitudeChar[position]];
        
        const char northing = northingStr[0];
        const char easting = eastingStr[0];
    
        switch (position) {
            
            // If the sentence is of type GLL
            case 0 : 
                return Position(latitudeStr, northing, longitudeStr, easting);
                break;
               
            // If the sentence is of type GGA
            case 1 : 
                if (fields[elevationUnit[position]] != "M") {
                    throw std::invalid_argument("Invalid argument, the units must be 'M' for metres.");
                }
                else {
                    return Position(latitudeStr, northing, longitudeStr, easting, elevationStr);
                }
                break;
            
            //if the sentence is of type RMC
            case 2: 
                return Position(latitudeStr, northing, longitudeStr, easting);
                break;
                
            // if the sentence is invalid
            default:
                throw std::invalid_argument("This is not a valid sentence type.");
                break;
        }
                
    }

    
    std::vector<Position> routeFromNMEALog(const std::string & filepath) {
        // ifstream t
        std::ifstream logFile;
        logFile.open(filepath);
        
        // Initialising a vector that will store positions
        std::vector<Position> routeFromLog;
        
        if (!logFile) {
            throw std::invalid_argument("Log file does not exist");
        }
        // For every sentence in the file (each comma separated value) the function will check if it is a valid sentence
        // If it is a valid sentence it will decompose it and extract the position, then saving this to the routeFromLog vector.
        std::string sentence;
        while(std::getline(logFile, sentence)) {
            const std::string fileSentence = sentence;
            if (GPS::isValidSentence(sentence)) {
                NMEAPair decomposedSentence = decomposeSentence(fileSentence);
                routeFromLog.push_back(extractPosition(decomposedSentence));
            }
            else {
                continue;
            }
        
        }
        
        return routeFromLog;
    }

}
