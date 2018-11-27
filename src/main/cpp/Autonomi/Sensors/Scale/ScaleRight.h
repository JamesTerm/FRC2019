/*
 * ScaleRight.h
 *
 *  Created on: Mar 27, 2018
 *      Author: Dylann Ruiz
 */

#ifndef SRC_AUTONOMI_SENSORS_SCALE_SCALERIGHT_H_
#define SRC_AUTONOMI_SENSORS_SCALE_SCALERIGHT_H_

#include "Config/ActiveCollection.h"
#include <string>

using namespace std;
using namespace Configuration;

namespace Autonomi{
class ScaleRight
{
	private:
		ActiveCollection *activeCollection;
		string gameData;

	public:
		ScaleRight(ActiveCollection *_activeCollection);
		void Start();
		virtual ~ScaleRight(){};
};
}

#endif
