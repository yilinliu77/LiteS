#pragma once

#ifndef CCAMERA_H
#define CCAMERA_H

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <vector>

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum Camera_Movement {
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT
};

// Default camera values
const float YAW = 0.f;
const float PITCH = 0.0f;
const float ROLL = 0.0f;
const float SENSITIVTY = 0.005f;
const float ZOOM = 45.0f;

// An abstract camera class that processes input and calculates the corresponding Eular Angles, Vectors and Matrices for use in OpenGL
class CCamera {
public:
	// Camera Attributes
	glm::vec3 Position;
	glm::vec3 Front;
	glm::vec3 Up;
	glm::vec3 Right;
	glm::vec3 WorldUp;
	glm::vec3 WorldCenter;
	// Eular Angles
	float Yaw;
	float Pitch;
	float Roll;
	// Camera options
	float MovementSpeed;
	float MouseSensitivity;
	float Zoom;

	// Constructor with vectors
	CCamera(float vCameraSpeed,glm::vec3 position = glm::vec3(0.0f, 0.0f, 3.0f)
		, glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f)) : Front(glm::vec3(0.0f, 0.0f, -1.0f))
				, MovementSpeed(vCameraSpeed), MouseSensitivity(SENSITIVTY)
				, Zoom(ZOOM), WorldCenter(glm::vec3(0.f,0.f,0.f))
				,Yaw(YAW),Pitch(PITCH),Roll(ROLL){
		Position = position;
		Front = glm::normalize(WorldCenter - Position);
		Up = up;
		Right = glm::normalize(glm::cross(Front, Up));

		updateCameraVectors();
	}
	// Constructor with scalar values
	/*CCamera(float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw, float pitch) : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVTY), Zoom(ZOOM) {
		Position = glm::vec3(posX, posY, posZ);
		WorldUp = glm::vec3(upX, upY, upZ);
		Yaw = yaw;
		Pitch = pitch;
		updateCameraVectors();
	}
	*/
	// Returns the view matrix calculated using Eular Angles and the LookAt Matrix
	glm::mat4 GetViewMatrix() {
		return glm::lookAt(Position, Position + Front, Up);
	}

	// Processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
	void ProcessKeyboard(Camera_Movement direction, float deltaTime) {
		float velocity = MovementSpeed * deltaTime;
		if (direction == FORWARD)
			Position += Front * velocity;
		if (direction == BACKWARD)
			Position -= Front * velocity;
		if (direction == LEFT)
			Position -= Right * velocity;
		if (direction == RIGHT)
			Position += Right * velocity;
	}

	// Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
	void ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true) {
		xoffset *= MouseSensitivity;
		yoffset *= MouseSensitivity;

		glm::vec2 direction(xoffset, yoffset);

		Yaw = glm::dot(direction,glm::vec2(1.f,0.f));
		Pitch = glm::dot(direction, glm::vec2(0.f, 1.f));
		Roll = glm::dot(direction, glm::vec2(-1.f, -1.f));

		// Update Front, Right and Up Vectors using the updated Eular angles
		updateCameraVectors();
	}

	// Processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
	void ProcessMouseScroll(float yoffset) {
		//if (Zoom >= 1.0f && Zoom <= 45.0f)
		//	Zoom -= yoffset;
		//if (Zoom <= 1.0f)
		//	Zoom = 1.0f;
		//if (Zoom >= 45.0f)
		//	Zoom = 45.0f;

		Position += Front * yoffset * 10.0f *SENSITIVTY;

	}

private:
	// Calculates the front vector from the Camera's (updated) Eular Angles
	void updateCameraVectors() {
		// Calculate the new Front vector
		glm::vec3 front;
		front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
		front.y = sin(glm::radians(Pitch));
		front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
		Front = WorldCenter - Position;
		//glm::quat rotateionQuat = glm::quat(glm::vec3(Pitch, -Yaw, 0));
		//Front = rotateionQuat*glm::vec4(Front,1.f);
		Front = glm::rotate(glm::mat4(1.0f), -Yaw, Up)*glm::vec4(Front, 1.0f);
		Front = glm::rotate(glm::mat4(1.0f), Pitch, Right)*glm::vec4(Front, 1.0f);
		//Front = glm::rotate(glm::mat4(1.0f), Roll, glm::vec3(0.f, 0.f, 1.0f))*glm::vec4(Front,1.0f);
		Position = WorldCenter - Front;
		Front = glm::normalize(Front);
		Up = glm::normalize(glm::cross(Right, Front));
		Right = glm::normalize(glm::cross(Front, Up));  // Normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
	}
};

#endif