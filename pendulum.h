#include <utility>
#include <vector>
#include <SFML/Graphics.hpp>

class Pendulum
{


public:
	//Pivot coords
	sf::Vector2f pivot;

	//Length of rod connected to pivot
	double l1;
	double l2;

	//Angle of pendulum from vertical
	double angle1;
	double angle2;

	double aVel1;
	double aVel2;

	//Masses of pendulums
	double m1;
	double m2;

	sf::CircleShape bob1;
	sf::CircleShape bob2;

	sf::Vertex line1[2];
	sf::Vertex line2[2];

	sf::Color trailColor;
	std::vector<sf::Vertex> trail;

public:
	void update();
	Pendulum(sf::Vector2f pivot, double l1, double l2, double m1, double m2,
		double angle1, double angle2, sf::Color bobColor, sf::Color lineColor, sf::Color trailColor);
	double calcEnergy();
};
