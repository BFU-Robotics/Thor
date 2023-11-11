#pragma once
#include "GyverStepper2.h"

namespace thor {

    struct Pins
    {
        int steps, stepPin, dirPin;
    };

    class MotorManager
    {
        private:
            int m_n;
            GStepper2<STEPPER2WIRE>* m_motors[10];
            bool m_dir[10] = {1};

        public:
            MotorManager(int n) : m_n(n) 
            {
                
            }

            ~MotorManager()
            {
                for(int i=0;i<m_n;i++)
                {
                    delete m_motors[i];
                }
            }

            void Setup(Pins* pins)
            {
                for(int i=0;i<m_n;i++)
                {
                    m_motors[i] = new GStepper2<STEPPER2WIRE>(pins[i].steps, pins[i].stepPin, pins[i].dirPin);
                    m_motors[i]->enable();
                    m_motors[i]->setMaxSpeed(200);     // скорость движения к цели
                    m_motors[i]->setAcceleration(400); // ускорение
                    m_motors[i]->setTarget(100);       // цель
                }
            }

            void Job()
            {
                for(int i=0;i<m_n;i++)
                {
                    m_motors[i]->tick();   // мотор асинхронно крутится тут
                    // если приехали
                    if (m_motors[i]->ready()) {
                      m_dir[i] = !m_dir[i];   // разворачиваем
                      m_motors[i]->setTarget(m_dir[i] * 600); // едем в другую сторону
                    }
                }
            }
    };


}