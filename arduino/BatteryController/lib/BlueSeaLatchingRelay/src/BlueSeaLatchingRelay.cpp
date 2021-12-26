/*
  Relay Library for controlling Latching Relays on a Marine or Home battery system.
  Created by Clément Lambert, 6th march 2018.
  Released into the public domain.
*/

#include "BlueSeaLatchingRelay.h"

#define DEBUGR 0

/**
 * Constructor
 */
BlueSeaLatchingRelay::BlueSeaLatchingRelay()
{
}

void BlueSeaLatchingRelay::setClosed()
{
	this->isReadyToOpen = 0;

	if (this->getState() != BlueSeaLatchingRelay::RELAY_CLOSE)
	{
#if DEBUGR == 1
		Serial.print(this->name);
		Serial.println(F(" closing"));
#endif
		BlueSeaLatchingRelay::customWrite(this->closePin, 1);

		delay(latchingDurationTime);

		BlueSeaLatchingRelay::customWrite(this->closePin, 0);

		// delay to be sure that 2 relays can't be activated at the same time (causing fuse blow)
		delay(800);

		if (this->getState() == BlueSeaLatchingRelay::RELAY_CLOSE)
		{
			// les états demandant l'ouverture du relais sont remis à 0
			this->isForceToClose = 0;
			this->isReadyToClose = 0;

#if DEBUGR == 1
			Serial.print(this->name);
			Serial.println(F(" closed"));
#endif
		}
		else
		{
#if DEBUGR == 1
			Serial.print(F("Error36, can't close "));
			Serial.println(this->name);
			Serial.print(F("Pin : "));
			Serial.println(this->closePin);
			Serial.print(F("State : "));
			Serial.println(this->getState());
			Serial.print(F("RELAY_CLOSE : "));
			Serial.println(BlueSeaLatchingRelay::RELAY_CLOSE);
#endif
		}
	}
}

void BlueSeaLatchingRelay::setOpened()
{
#if DEBUGR == 1
	Serial.print(this->getState());
	Serial.print(" :: ");
	Serial.println(BlueSeaLatchingRelay::RELAY_OPEN);
#endif

	if (this->getState() != BlueSeaLatchingRelay::RELAY_OPEN)
	{

		// excitation du relais
		BlueSeaLatchingRelay::customWrite(this->openPin, 1);

#if DEBUGR == 1
		Serial.print(this->name);
		Serial.println(F(" opening"));
#endif
		delay(latchingDurationTime);

		// arrêt de l'excitation
		BlueSeaLatchingRelay::customWrite(this->openPin, 0);

		// delay to be sure that 2 relays can't be activated at the same time (causing fuse blow)
		delay(800);

		if (this->getState() == BlueSeaLatchingRelay::RELAY_OPEN)
		{
			// Le relais est marqué comme ouvert (non passant)
			// et les états demandant l'ouverture du relais sont remis à 0
			this->isReadyToOpen = 0;
			this->isForceToOpen = 0;
			this->waitingForOpening = false;

#if DEBUGR
			Serial.print(this->name);
			Serial.println(F(" opened"));
#endif
		}
		else
		{
#if DEBUGR
			Serial.println(F("Error63, can't open "));
			Serial.print(this->name);
#endif

			// Serial.print("openPin : ");
			// Serial.println(this->openPin);
		}
	}
}

void BlueSeaLatchingRelay::setReadyToClose()
{
#if DEBUGR
	Serial.print(this->name);
	Serial.println(" RDY CLOSE");
#endif
	this->isReadyToClose = 1;
}

void BlueSeaLatchingRelay::setReadyToOpen()
{
#if DEBUGR
	Serial.print(this->name);
	Serial.println(" RDY OPEN");
#endif
	this->isReadyToOpen = 1;
}

void BlueSeaLatchingRelay::forceToClose()
{
#if DEBUGR
	Serial.print(this->name);
	Serial.println(" FORCE CLOSE");
#endif
	this->isForceToClose = 1;
	this->setClosed();
}
void BlueSeaLatchingRelay::forceToOpen()
{
#if DEBUGR
	Serial.print(this->name);
	Serial.println(" FORCE OPEN");
#endif
	this->isForceToOpen = 1;
	this->setOpened();
}

/*
 * Apply ready actions at the end of cycle
 * ForceOpen or ForceClose have priority over readyToClose or readyToOpen
 * ForceOpen has also priority over ForceClose
 */
void BlueSeaLatchingRelay::applyReadyActions()
{
	if ((this->isForceToOpen != 1) && (this->isForceToClose != 1))
	{
		// si le relais était en attente d'être ouvert
		// on indique au relais qu'il peut être ouvert
		// isReadyToOpen peut être = 0 si on est rentré dans une nouvelle boucle et dans ce cas les états sont automatiquement remis à 0
		// if (this->waitingForOpening && this->delayBeforeOpening > 0)
		// {
		// 	this->isReadyToOpen = 1;
		// }

		if (this->isReadyToOpen == 1 && (this->getState() == BlueSeaLatchingRelay::RELAY_CLOSE))
		{
			// si un délais doit être appliqué
			// avant l'ouverture du relais
			if (this->delayBeforeOpening > 0)
			{

				// initialisation du compte à rebours
				if (this->waitingForOpening == false)
				{
					this->delayBeforeOpeningStartMillis = millis();
					this->waitingForOpening = true;
#if DEBUGR == 1
					Serial.println(F("Wop.."));
#endif
				}

				// Si un compte à rebours à déjà été lancé
				if (this->delayBeforeOpeningStartMillis != 0)
				{

					// unsigned long lapsTime = millis() - this->delayBeforeOpeningStartMillis;
					// Time to open the relay
					if ((millis() - this->delayBeforeOpeningStartMillis) >= this->delayBeforeOpening)
					{
						this->setOpened();
						this->delayBeforeOpeningStartMillis = 0;
						this->waitingForOpening = false;
#if DEBUGR
						Serial.println(F("wEnd."));
#endif
					}
					else
					{
#if DEBUGR
						Serial.println(millis() - this->delayBeforeOpeningStartMillis);
#endif
					}
				}
			}
			else
			{
				this->setOpened();
			}
		}

		else if (this->isReadyToClose == 1)
			this->setClosed();
	}
}

void BlueSeaLatchingRelay::startCycle()
{
	// this->isReadyToOpen = 0;
	// this->isReadyToClose = 0;
	// this->isForceToOpen = 0;
	// this->isForceToClose = 0;
}

/*
* Getting relay state
* comparaison with the logic pin and the digital state from BlueSea
* !! Relay value = HIGH if relay open
*/
byte BlueSeaLatchingRelay::getState()
{

	if (this->statePin)
	{

		int digitalState;

		// Détection de l'état du Relais
		if (BlueSeaLatchingRelay::isAnalogPin(this->statePin))
		{ // analog pin
			digitalState = analogRead(this->statePin);
			digitalState = (digitalState > 511) ? HIGH : LOW;
		}
		else
		{
			digitalState = digitalRead(this->statePin);
		}

		// L'état du capteur est inversé
		//quand l'arduino = HIGH, réellement le relais est Ouvert (opened)
		// On remet l'état comme il faut
		digitalState = !digitalState;

		// Si le capteur détecte que le relais est passant (closed)
		// mais que l'état du relais dans le code ne l'est pas.
		// l'état du relais est établis sur passant (closed)
		if (digitalState == HIGH && (this->state != BlueSeaLatchingRelay::RELAY_CLOSE))
		{
			this->state = BlueSeaLatchingRelay::RELAY_CLOSE;
		}

		// A l'inverse, si le capteur détecte que le relais est ouvert (Opened)
		// mais que l'état du relais dans le code ne l'est pas.
		// l'état du relais est établis sur ouvert (Opened)
		if (digitalState == LOW && (this->state != BlueSeaLatchingRelay::RELAY_OPEN))
		{
			this->state = BlueSeaLatchingRelay::RELAY_OPEN;
		}
	}

	return this->state;
}

boolean BlueSeaLatchingRelay::isAnalogPin(uint8_t pin)
{
	if (pin == 20 || pin == 21)
	{
		return true;
	}

	return false;
}

void BlueSeaLatchingRelay::customWrite(uint8_t pin, uint8_t val)
{
	if (BlueSeaLatchingRelay::isAnalogPin(pin))
	{
		analogWrite(pin, val);
	}
	else
	{
		digitalWrite(pin, val);
	}
}
