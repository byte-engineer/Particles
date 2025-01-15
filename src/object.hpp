#pragma once

namespace particle
{
    class Object
    {
    public:
        Object(vec2 pos, float area, float density)
        : m_pos(pos), m_area(area), m_density(density)
        {
            s_count++;
            m_acceleratation = vec2::zeros();
            m_Forces.reserve(5);
            m_isStatic = false;
            do
            {
                m_velocity = vec2::random(-1.f, 1.f);
            } while (m_velocity.len() <= 1);
            
            m_velocity = vec2::random(-200.f, 200.f);
        }
        static int s_count;


        void addForce(vec2 force) { m_Forces.push_back(force); }
        float getMass()            { return m_area * m_density; }
        float getMomentum()        { return getMass() * m_velocity.len(); }
        vec2 getPosition()         { return m_pos; }

        float getKE()
        {
            float mag = m_velocity.len();
            return 0.5f * getMass() * (mag*mag);
        }

        vec2 getAllForces()
        {
            vec2 allForces = vec2::zeros();
            for (vec2 force : m_Forces)
            {
                allForces += force;
            }
            return allForces;
        }

        void setVelocity(vec2 velosity)
        {
            m_velocity = velosity;
        }

        vec2 getVelocity()
        {
            return m_velocity;
        }

        void updatePosition(float ts)
        {
            m_velocity += m_acceleratation * ts;
            m_pos += m_velocity * ts;
        }

        virtual void draw(Color color) = 0;
        virtual void moveTo(vec2 location) = 0;
        virtual void move(vec2 dPos) = 0;

        void dispState()
        {
            std::cout << "Position       :" << m_pos.x << ", " << m_pos.y << std::endl;
            std::cout << "Velosity       :" << m_velocity.x << " ," << m_velocity.y << std::endl;
            std::cout << "Acceleration   :" << m_acceleratation.x << ", " << m_acceleratation.x << std::endl;
            std::cout << "Mass           :" << getMass() << std::endl;
            std::cout << "Kinatic Energy :" << getKE() << std::endl;
            std::cout << "Momentum       :" << getMomentum() << std::endl;
        }


        ~Object()
        {
            s_count--;
        }

    protected:
        vec2 m_pos;
        vec2 m_velocity;
        vec2 m_acceleratation;
        float m_area;                   // deal with this as a volume.
        float m_density;
        bool m_isStatic;
        std::vector<vec2> m_Forces;
    }; // object

    int Object::s_count = 0;



    class Circle : public Object
    {
    public:
        Circle(vec2 pos,float raduis,float density = 1.0f)
        :Object(pos, PI*raduis*raduis, density), m_radius(raduis)
        {
            // ...
        }

        void draw(Color color) override
        {
            Draw::CircleWithStrok(m_pos, m_radius, 1, color, BLACK);
        }

        float getRaduis()
        {
            return m_radius;
        } 

        void moveTo(vec2 location) override
        {
            m_pos = location;   
        }

        void move(vec2 dPos) override
        {
            m_pos += dPos;
        }

        void updatePosition(float ts)
        {
            move(m_velocity*ts);
        }

        void sideCollisions(vec2 winSize, bool roof=true)
        {
            if (m_pos.x - m_radius <= 0)
                m_pos.x = m_radius;
            else if (m_pos.x + m_radius >= winSize.x)
                m_pos.x = winSize.x - m_radius;

            if (m_pos.y - m_radius <= 0 && roof)
                m_pos.y = m_radius;
            else if (m_pos.y + m_radius >= winSize.y )
                m_pos.y = winSize.y - m_radius;


            // Handle collisions with left or right walls
            if (m_pos.x - m_radius <= 0 || m_pos.x + m_radius >= winSize.x)
            {
                m_velocity.x *= -1; 
            }

            // Handle collisions with top or bottom walls
            if ((m_pos.y - m_radius <= 0) && roof || m_pos.y + m_radius >= winSize.y)
            {
                m_velocity.y *= -1; 
            }
        
        }

        bool isCollide(particle::Circle& other)
        {
            if ( !(other.getPosition() == this->getPosition()) )
            {
                float d = getPosition().distance(other.getPosition());
                if (d < (getRaduis() + other.getRaduis()) )
                    return true;
            }
            return false;
        }


        void updateCollisionVelocity(particle::Circle& other, float coefficientOfRestitution)
        {
            float m1 = this->getMass();
            float m2 = other.getMass();

            vec2 pos1 = this->getPosition();
            vec2 pos2 = other.getPosition();

            vec2 v1 = this->getVelocity();
            vec2 v2 = other.getVelocity();

            vec2 relativePos = pos2 - pos1;
            vec2 relativeVelocity = v2 - v1;

            float distance = relativePos.len();
            if (distance < 1e-6f) distance = 1e-6f; // Avoid division by zero

            vec2 collisionAxis = relativePos / distance;

            float velocityAlongAxis = relativeVelocity.dot(collisionAxis);

            if (velocityAlongAxis > 0) return;

            // Impulse scalar
            float impulse = (-(1 + coefficientOfRestitution) * velocityAlongAxis) / (1 / m1 + 1 / m2);

            vec2 impulseVector = -impulse * collisionAxis;
            this->setVelocity(v1 + impulseVector / m1);
            other.setVelocity(v2 - impulseVector / m2);

            float overlap = this->getRaduis() + other.getRaduis() - distance;
            if (overlap > 0)
            {
                vec2 correction = collisionAxis * (overlap / 2);
                this->moveTo(pos1 - correction);
                other.moveTo(pos2 + correction);
            }
        }

        void setColor(Color col)
        {
            m_color = col;
        }

        Color getColor() 
        {
            return m_color;
        }


        vec2 getAcceleration()
        {
            return m_acceleratation;
        }

        vec2 setAcceleration(vec2 acceleration)
        {
            m_acceleratation = acceleration;
        }

        void updateVelocity(float ts)
        {
            m_acceleratation = getAllForces() / getMass();
            m_velocity += ts * m_acceleratation;
        }

        private:
            float m_radius;
            Color m_color;

    }; // Circle 

} // paricle