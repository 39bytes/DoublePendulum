#include "Pendulum.h"
#include <math.h>
#include <utility>
#include <array>

//Gravity
const double g = 9.81;
//Time step
const double dt = 0.02;

//The second derivative of theta1 with respect to time (angular acceleration)
double deriv1(double theta1, double theta2, double v1, double v2, double l1, double l2, double m1, double m2)
{
	double top = -g * (2 * m1 + m2) * sin(theta1) - m2 * g * sin(theta1 - 2 * theta2) - 2 * sin(theta1 - theta2) * m2
		* (v2 * v2 * l2 + v1 * v1 * l1 * cos(theta1 - theta2));
	double bottom = l1 * (2 * m1 + m2 - m2 * cos(2 * theta1 - 2 * theta2));
	return top / bottom;
}

//The second derivative of theta2 with respect to time (angular acceleration)
double deriv2(double theta1, double theta2, double v1, double v2, double l1, double l2, double m1, double m2)
{
	double top = 2 * sin(theta1 - theta2) *
		(v1 * v1 * l1 * (m1 + m2) + g * (m1 + m2) * cos(theta1) + v2 * v2 * l2 * m2 * cos(theta1 - theta2));
	double bottom = l2 * (2 * m1 + m2 - m2 * cos(2 * theta1 - 2 * theta2));
	return top / bottom;
}

// Returns the total energy in the system
double Pendulum::calcEnergy()
{
	double potential = (m1 + m2) * l1 * g * (1 - cos(angle1)) + m2 * l2 * g * (1 - cos(angle2));
	double kinetic = 0.5 * m1 * (l1 * aVel1) * (l1 * aVel1) + 0.5 * m2 * ((l1 * aVel1) * (l1 * aVel1) + (l2 * aVel2) * (l2 * aVel2) +
		2 * l1 * l2 * aVel1 * aVel2 * cos(angle1 - angle2));
	return potential + kinetic;
}

//Updates the pendulum angle and calculates
//the new position of the bob in cartesian coordinates.
void Pendulum::update()
{
	// Runge-Kutta method to solve the system of differential equations
	double k11 = aVel1 * dt;
	double k21 = aVel2 * dt;
	double k31 = deriv1(angle1, angle2, aVel1, aVel2, l1, l2, m1, m2) * dt;
	double k41 = deriv2(angle1, angle2, aVel1, aVel2, l1, l2, m1, m2) * dt;

	double k12 = (aVel1 + k31 / 2) * dt;
	double k22 = (aVel2 + k41 / 2) * dt;
	double k32 = deriv1(angle1 + k11 / 2, angle2 + k21 / 2, aVel1 + k31 / 2, aVel2 + k41 / 2, l1, l2, m1, m2) * dt;
	double k42 = deriv2(angle1 + k21 / 2, angle2 + k21 / 2, aVel1 + k31 / 2, aVel2 + k41 / 2, l1, l2, m1, m2) * dt;

	double k13 = (aVel1 + k32 / 2) * dt;
	double k23 = (aVel2 + k42 / 2) * dt;
	double k33 = deriv1(angle1 + k12 / 2, angle2 + k22 / 2, aVel1 + k32 / 2, aVel2 + k42 / 2, l1, l2, m1, m2) * dt;
	double k43 = deriv2(angle1 + k22 / 2, angle2 + k22 / 2, aVel1 + k32 / 2, aVel2 + k42 / 2, l1, l2, m1, m2) * dt;

	double k14 = (aVel1 + k33) * dt;
	double k24 = (aVel2 + k43) * dt;
	double k34 = deriv1(angle1 + k13, angle2 + k23, aVel1 + k33, aVel2 + k43, l1, l2, m1, m2) * dt;
	double k44 = deriv2(angle1 + k23, angle2 + k23, aVel1 + k33, aVel2 + k43, l1, l2, m1, m2) * dt;

	angle1 += (k11 + 2 * k12 + 2 * k13 + k14) / 6.0;
	angle2 += (k21 + 2 * k22 + 2 * k23 + k24) / 6.0;
	aVel1 += (k31 + 2 * k32 + 2 * k33 + k34) / 6.0;
	aVel2 += (k41 + 2 * k42 + 2 * k43 + k44) / 6.0;

	// Dampening factor
	aVel1 *= 0.999;
	aVel2 *= 0.999;

	// Calculate new bob positions
	double newX1 = pivot.x + l1 * 50 * sin(angle1);
	double newY1 = pivot.y + l1 * 50 * cos(angle1);

	sf::Vector2f bob1Pos = sf::Vector2f((float)newX1, (float)newY1);

	bob1.setPosition(bob1Pos);

	// Draw line from pivot to bob
	line1[0].position = pivot;
	line1[1].position = bob1Pos;

	// Do the same for the second one
	double newX2 = newX1 + l2 * 50 * sin(angle2);
	double newY2 = newY1 + l2 * 50 * cos(angle2);

	sf::Vector2f bob2Pos = sf::Vector2f((float)newX2, (float)newY2);

	bob2.setPosition(bob2Pos);
	line2[0].position = bob1Pos;
	line2[1].position = bob2Pos;

	//Add trail
	trail.insert(trail.begin(), (sf::Vertex(bob2Pos, trailColor)));
	if (trail.size() > 60)
	{
		trail.pop_back();
	}
}

Pendulum::Pendulum(sf::Vector2f pivot, double l1, double l2, double m1, double m2,
				   double angle1, double angle2, sf::Color bobColor, sf::Color lineColor, sf::Color trailColor)
{
	this->angle1 = angle1;
	this->angle2 = angle2;
	aVel1 = 0;
	aVel2 = 0;

	this->pivot = pivot;
	this->l1 = l1;
	this->l2 = l2;
	this->m1 = m1;
	this->m2 = m2;

	this->trailColor = trailColor;

	bob1 = sf::CircleShape(5.0f);
	bob1.setOrigin(5.0f, 5.0f);
	bob1.setFillColor(bobColor);

	bob2 = sf::CircleShape(5.0f);
	bob2.setOrigin(5.0f, 5.0f);
	bob2.setFillColor(bobColor);

	line1[0].color = lineColor;
	line1[1].color = lineColor;

	line2[0].color = lineColor;
	line2[1].color = lineColor;
}
