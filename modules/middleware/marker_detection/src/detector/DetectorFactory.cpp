/**
  * Detector factory class

  * Abstracts the creation of detectors, for use as position estimator
  * To add a new detector, add a line in the constuctor, with a new class
  *
  * April 2015 - Robin Hoogervorst - RaM
  */

#include <map>
#include "AbstractDetector.cpp"
#include "AprilDetector.cpp"
#include "DotsDetector.cpp"

class DetectorFactory {

private:
  std::map<std::string, AbstractDetector*> detectors;

public:

  DetectorFactory() {
    /*
     * Initialisation of a hashmap of detectors
     * To add a detector, add its alias and a new instance of it here
     *
     * This probably uses more memory than it should, but it's easier coding :)
     */
    detectors["APRIL"]  = new AprilDetector();
    detectors["DOTS"]   = new DotsDetector();
  }


  /**
    *  Destuctor, removes all classes that have been made, so our memory is free again :)
    */
  ~DetectorFactory() {
    std::map<std::string, AbstractDetector*>::iterator i;
    for (i = detectors.begin(); i != detectors.end(); i++){
        delete(i->second);
    }
  }

  /**
    * Returns an instance of an Abstract Detector implementation by using a search through our hashmap,
    * To be used for tag image detection
    */
  AbstractDetector* get(std::string detector_id, std::string calibrationFile) {
      std::map<std::string, AbstractDetector*>::iterator i = detectors.find(detector_id);
      if (i != detectors.end()) {
          i->second->parseCameraSettings(calibrationFile);
          return i->second;
      } else {
          return NULL;
      }
  }
};
