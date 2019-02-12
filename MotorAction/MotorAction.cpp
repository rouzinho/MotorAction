
// CEDAR INCLUDES
#include "MotorAction.h"
#include <cedar/processing/ExternalData.h> // getInputSlot() returns ExternalData
#include <cedar/auxiliaries/MatData.h> // this is the class MatData, used internally in this step
#include "cedar/auxiliaries/math/functions.h"
#include<cstdlib>
#include<ctime>

// SYSTEM INCLUDES

//----------------------------------------------------------------------------------------------------------------------
// constructors and destructor
//----------------------------------------------------------------------------------------------------------------------
MotorAction::MotorAction()
:
cedar::proc::Step(true),
mOutput(new cedar::aux::MatData(cv::Mat::zeros(1, 100, CV_32F))),
mSize(new cedar::aux::IntParameter(this, "Size",100)),
mAmplitude(new cedar::aux::DoubleParameter(this,"Amplitude",2.0)),
mSigma(new cedar::aux::DoubleParameter(this,"Sigma",3.0))
{
this->declareOutput("output", mOutput);
this->declareInput("ready",true);
this->declareInput("explore",true);
srand(static_cast<unsigned int>(clock()));
mGaussMatrixSizes.push_back(100);
mGaussMatrixSigmas.push_back(3.0);
mGaussMatrixCenters.push_back(25.0);
lock_action = 1;
lower_bound = 0;
upper_bound = 100;
//init the variable that will get the sensor value
dat = 0;

this->connect(this->mSize.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mSigma.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mAmplitude.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));

}
//----------------------------------------------------------------------------------------------------------------------
// methods
//----------------------------------------------------------------------------------------------------------------------
void MotorAction::compute(const cedar::proc::Arguments&)
{

  auto exp = boost::dynamic_pointer_cast<cedar::aux::ConstMatData>(this->getInput("explore"));
  auto ready_peak = boost::dynamic_pointer_cast<cedar::aux::ConstMatData>(this->getInput("ready"));
  exploration = cedar::aux::math::getMatrixEntry<double>(exp->getData(), 0, 0);
  ready = cedar::aux::math::getMatrixEntry<double>(ready_peak->getData(), 0, 0);
  std::cout << "ready : " << ready << '\n';

  if (exploration < 0.8)
  {
     if(ready > 0.8 && lock_action == 1)
     {
        mGaussMatrixCenters.clear();
        next_action = double(rand()) / (double(RAND_MAX) + 1.0)*100;
        std::cout << "next action " << next_action <<'\n';
        mGaussMatrixCenters.push_back(next_action);
        dat = 2.0;
        lock_action = 0;
     }
     if(ready < 0.8)
     {
       std::cout << "not ready " << '\n';
       dat = 0;
       lock_action = 1;
     }
  }
  this->mOutput->setData(cedar::aux::math::gaussMatrix(1,mGaussMatrixSizes,dat,mGaussMatrixSigmas,mGaussMatrixCenters,true));

}

void MotorAction::reCompute()
{
   mGaussMatrixSizes.clear();
   mGaussMatrixSigmas.clear();
   mGaussMatrixSizes.push_back(static_cast<int>(this->mSize->getValue()));
   mGaussMatrixSigmas.push_back(static_cast<double>(this->mSigma->getValue()));
   dat = static_cast<int>(this->mAmplitude->getValue());
}


void MotorAction::reName()
{

}

void MotorAction::reset()
{

	//ros::shutdown();

}
