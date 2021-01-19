/****************************** Header ******************************\
Class Name: DigitalInputItem inherits InputComponent
File Name:	DigitalInputItem.h
Summary: Abstraction for the WPIlib DigitalInput that extends to include
some helper and control methods.
Project:     BroncBotzFRC2019
Copyright (c) BroncBotz.
All rights reserved.

Author(s): Dylan Watson
Email: dylantrwatson@gmail.com
\********************************************************************/

#ifndef SRC_COMPONENTS_DIGITALINPUTITEM_H_
#define SRC_COMPONENTS_DIGITALINPUTITEM_H_

#include "InputComponent.h"

namespace Components
{
    class DigitalInputItem : public InputComponent
    {
        private:
	        DigitalInput *din;
	        int channel;

        public:
	        DigitalInputItem(){}
	        DigitalInputItem(int _channel, string name, bool Real);
	        bool GetBool();
	        virtual double Get() override;
	        virtual string GetName() override;
			virtual void DeleteComponent() override;
	        virtual ~DigitalInputItem();
    };
}

#endif /* SRC_COMPONENTS_DIGITALINPUTITEM_H_ */
