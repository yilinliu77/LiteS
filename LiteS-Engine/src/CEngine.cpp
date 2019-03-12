#include "CEngine.h"

#define STB_IMAGE_IMPLEMENTATION
#include"stb_image.h"
#include <thread>
#include "CDifferPass.h"
#include "CPointCloudComponent.h"
#include <imgui.h>
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void APIENTRY glDebugOutput(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam);

CEngine::CEngine():pointSize(4){ keyPreesed.assign(500, false); }

CEngine::~CEngine() {}

bool CEngine::initEngine(string configFile) {
	if (!this->__initScene(configFile)) {
		std::cout << "Scene init failed" << std::endl;
		return false;
	}
	if (!this->__initDLL()) {
		std::cout << "DLL init failed" << std::endl;
		return false;
	}
	if (!this->__readProperties(configFile)) {
		std::cout << "Properties init failed" << std::endl;
		return false;
	}
	return true;
}

void CEngine::runEngine() {
	this->m_Component->extraInit();
	//this->m_Component->extraAlgorithm();
	std::thread initThread(&CComponent::extraAlgorithm,this->m_Component);
	initThread.detach();
	std::cout << "done" << std::endl;

	while (!glfwWindowShouldClose(this->m_Window)) {
		float currentFrame = (float)glfwGetTime();
		m_deltaTime = currentFrame - m_lastFrame;
		m_lastFrame = currentFrame;
		handleInput(m_Window);

		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		//ImGui::ShowDemoWindow(&show_demo_window);

		{
			static float f = 0.0f;
			static int counter = 0;

			ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

			ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
			//ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state

			ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
			ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
			ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

			if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
				counter++;
			ImGui::SameLine();
			ImGui::Text("counter = %d", counter);

			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			ImGui::End();
		}

		ImGui::Render();

		this->m_Component->run();

		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(m_Window);
		glfwPollEvents();
	}
	
	
}

bool CEngine::isClicked(GLFWwindow * window,unsigned key) {
	if (glfwGetKey(window, key) == GLFW_PRESS) {
		keyPreesed[key] = true;
		return false;
	}
	if (keyPreesed[key] && glfwGetKey(window, key) == GLFW_RELEASE) {
		keyPreesed[key] = false;
		return true;
	}
}

void CEngine::handleInput(GLFWwindow * window) {
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		CEngine::m_Scene->m_Camera->ProcessKeyboard(FORWARD, m_deltaTime);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		CEngine::m_Scene->m_Camera->ProcessKeyboard(BACKWARD, m_deltaTime);
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		CEngine::m_Scene->m_Camera->ProcessKeyboard(LEFT, m_deltaTime);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		CEngine::m_Scene->m_Camera->ProcessKeyboard(RIGHT, m_deltaTime);

	if(isClicked(window,GLFW_KEY_F9)) {
		if(this->m_Component->m_shouldContinue) {
			this->m_Component->m_shouldContinue = false;
		}
		else
			this->m_Component->continueExtraAlgorithm();
	}
	if (isClicked(window, GLFW_KEY_F10))
		this->m_Component->stepExtraAlgorithm();
	if (isClicked(window, GLFW_KEY_EQUAL))
		glPointSize(pointSize += 1);
	if (isClicked(window, GLFW_KEY_MINUS)) {
		glPointSize(pointSize -= 1);
		if (pointSize < 0)
			pointSize = 0;
	}


}

void CEngine::switchMode(RenderMode v_mode) { m_mode = v_mode; }

void mouse_callback(GLFWwindow * window, double xpos, double ypos) {
	if(CEngine::m_mouseClicked) {
		float xoffset = (float)xpos - CEngine::m_lastX;
		float yoffset = CEngine::m_lastY - (float)ypos; // reversed since y-coordinates go from bottom to top

		CEngine::m_lastX = (float)xpos;
		CEngine::m_lastY = (float)ypos;

		CEngine::m_Scene->m_Camera->ProcessMouseMovement(xoffset, yoffset);
	}
	
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		CEngine::m_mouseClicked = true;
		double xpos;
		double ypos;
		glfwGetCursorPos(window, &xpos, &ypos);
		CEngine::m_lastX = (float)xpos;
		CEngine::m_lastY = (float)ypos;
	}
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
		CEngine::m_mouseClicked = false;
	}
}

void scroll_callback(GLFWwindow * window, double xoffset, double yoffset) {
	CEngine::m_Scene->m_Camera->ProcessMouseScroll((float)yoffset);
}

void APIENTRY glDebugOutput(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar * message, const void * userParam) {
	if (id == 131169 || id == 131185 || id == 131218 || id == 131204) return; // ignore these non-significant error codes

	std::cout << "---------------" << std::endl;
	std::cout << "Debug message (" << id << "): " << message << std::endl;

	switch (source) {
		case GL_DEBUG_SOURCE_API: std::cout << "Source: API";
			break;
		case GL_DEBUG_SOURCE_WINDOW_SYSTEM: std::cout << "Source: Window System";
			break;
		case GL_DEBUG_SOURCE_SHADER_COMPILER: std::cout << "Source: Shader Compiler";
			break;
		case GL_DEBUG_SOURCE_THIRD_PARTY: std::cout << "Source: Third Party";
			break;
		case GL_DEBUG_SOURCE_APPLICATION: std::cout << "Source: Application";
			break;
		case GL_DEBUG_SOURCE_OTHER: std::cout << "Source: Other";
			break;
	}
	std::cout << std::endl;

	switch (type) {
		case GL_DEBUG_TYPE_ERROR: std::cout << "Type: Error";
			break;
		case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: std::cout << "Type: Deprecated Behaviour";
			break;
		case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR: std::cout << "Type: Undefined Behaviour";
			break;
		case GL_DEBUG_TYPE_PORTABILITY: std::cout << "Type: Portability";
			break;
		case GL_DEBUG_TYPE_PERFORMANCE: std::cout << "Type: Performance";
			break;
		case GL_DEBUG_TYPE_MARKER: std::cout << "Type: Marker";
			break;
		case GL_DEBUG_TYPE_PUSH_GROUP: std::cout << "Type: Push Group";
			break;
		case GL_DEBUG_TYPE_POP_GROUP: std::cout << "Type: Pop Group";
			break;
		case GL_DEBUG_TYPE_OTHER: std::cout << "Type: Other";
			break;
	}
	std::cout << std::endl;

	switch (severity) {
		case GL_DEBUG_SEVERITY_HIGH: std::cout << "Severity: high";
			break;
		case GL_DEBUG_SEVERITY_MEDIUM: std::cout << "Severity: medium";
			break;
		case GL_DEBUG_SEVERITY_LOW: std::cout << "Severity: low";
			break;
		case GL_DEBUG_SEVERITY_NOTIFICATION: std::cout << "Severity: notification";
			break;
	}
	std::cout << std::endl;
	std::cout << std::endl;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}

bool CEngine::__initDLL() {
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);

	// glfw window creation
	// --------------------
	m_Window = glfwCreateWindow(this->m_Scene->m_WindowWidth, this->m_Scene->m_WindowHeight, "LearnOpenVR-Cube", NULL, NULL);
	if (m_Window == NULL) {
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return false;
	}

	glfwMakeContextCurrent(m_Window);
	glfwSetCursorPosCallback(m_Window, mouse_callback);
	glfwSetScrollCallback(m_Window, scroll_callback);
	glfwSetFramebufferSizeCallback(m_Window, framebuffer_size_callback);
	glfwSetMouseButtonCallback(m_Window, mouse_button_callback);
	// tell GLFW to capture our mouse
	glfwSetInputMode(m_Window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

	//init glew
	glewExperimental = GL_FALSE;
	GLenum nGlewError = glewInit();
	if (nGlewError != GLEW_OK) {
		printf("%s - Error initializing GLEW! %s\n", __FUNCTION__, glewGetErrorString(nGlewError));
		return false;
	}
	GLint flags;
	glGetIntegerv(GL_CONTEXT_FLAGS, &flags);
	if (flags & GL_CONTEXT_FLAG_DEBUG_BIT) {
		glEnable(GL_DEBUG_OUTPUT);
		glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS); // makes sure errors are displayed synchronously
		glDebugMessageCallback(glDebugOutput, nullptr);
		glDebugMessageControl(GL_DEBUG_SOURCE_API, GL_DEBUG_TYPE_ERROR, GL_DEBUG_SEVERITY_HIGH, 0, NULL, GL_TRUE);
	} else
		return false;

	glPointSize(pointSize);

	/*
	 * Imgui setup
	 */
	 // Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();

	// Setup Platform/Renderer bindings
	ImGui_ImplGlfw_InitForOpenGL(m_Window, true);
	ImGui_ImplOpenGL3_Init("#version 330");


	return true;
}

bool CEngine::__readProperties(string configFile) {
	stringstream stream;
	XMLDocument doc;
	doc.LoadFile(configFile.c_str());
	XMLNode *node = 0;

	node = doc.FirstChildElement("config")->FirstChildElement("Texture");//
	while (node&&!strcmp(node->ToElement()->Value(), "Texture")) {
		stream << node->FirstChildElement("Name")->GetText();
		string name;
		stream >> name;
		stream.clear();
		stream.str("");

		GLuint tex = 0;
		glGenTextures(1, &tex);

		//Source
		if (!node->FirstChildElement("File")) {
			//Texture Target
			const char* InternalFormatChar = node->FirstChildElement("INTERNAL_FORMAT")->GetText();
			int InternalFormatInt = -1;
			if (!strcmp(InternalFormatChar, "RGBA"))
				InternalFormatInt = GL_RGBA;
			else if (!strcmp(InternalFormatChar, "RGB"))
				InternalFormatInt = GL_RGB;
			else if (!strcmp(InternalFormatChar, "R"))
				InternalFormatInt = GL_RED;
			else {
				cout << "Can't resolve texture internal format" << endl;
				return false;
			}

			//Format
			int FormatInt = -1;
			if (node->FirstChildElement("FORMAT")) {
				const char* FormatChar = node->FirstChildElement("FORMAT")->GetText();
				if (!strcmp(FormatChar, "RGBA_16F"))
					FormatInt = GL_RGBA16F;
				else if (!strcmp(FormatChar, "RGB_16F"))
					FormatInt = GL_RGB16F;
				else if (!strcmp(FormatChar, "R_16F"))
					FormatInt = GL_R16F;
				else {
					cout << "Can't resolve texture format" << endl;
					return false;
				}
			}
			else {
				FormatInt = InternalFormatInt;
			}
			

			//Width and Height
			int RenderWidth, RenderHeight;
			stream << node->FirstChildElement("WIDTH")->GetText();
			stream >> RenderWidth;
			stream.clear();
			stream.str("");
			stream << node->FirstChildElement("HEIGHT")->GetText();
			stream >> RenderHeight;
			stream.clear();
			stream.str("");

			//Init the Texture
			glBindTexture(GL_TEXTURE_2D, tex);
			glTexImage2D(GL_TEXTURE_2D, 0, FormatInt, RenderWidth, RenderHeight, 0, InternalFormatInt, GL_UNSIGNED_BYTE, NULL);

			//Wrap
			if (node->FirstChildElement("WARP")) {
				const char* WarpOptions = node->FirstChildElement("WARP")->GetText();
				if (!strcmp(WarpOptions, "REPEAT")) {
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
				}
				else if (!strcmp(WarpOptions, "CLAMP_TO_EDGE")) {
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
				}
				else {
					cout << "Can't resolve texture wrap type" << endl;
					return false;
				}
			}
			else {
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			}

			//Filter
			if (node->FirstChildElement("FILTER")) {
				const char* FilterOptions = node->FirstChildElement("FILTER")->GetText();
				if (!strcmp(FilterOptions, "NEAREST")) {
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
				} else if (!strcmp(FilterOptions, "LINEAR")) {
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				} else {
					cout << "Can't resolve texture filter type" << endl;
					return false;
				}
			} else {
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			}
			
			glBindTexture(GL_TEXTURE_2D, 0);
		} else {
			stream << node->FirstChildElement("File")->GetText();
			string filename;
			stream >> filename;
			stream.clear();
			stream.str("");

			int width, height, nrComponents;

			unsigned char *data = stbi_load(filename.c_str(), &width, &height, &nrComponents, 0);
			if (data) {
				GLenum format;
				if (nrComponents == 1)
					format = GL_RED;
				else if (nrComponents == 3)
					format = GL_RGB;
				else if (nrComponents == 4)
					format = GL_RGBA;

				glBindTexture(GL_TEXTURE_2D, tex);
				glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
				glGenerateMipmap(GL_TEXTURE_2D);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

				glBindTexture(GL_TEXTURE_2D, 0);
				stbi_image_free(data);
			} else {
				std::cout << "Texture failed to load at path " << std::endl;
				stbi_image_free(data);
			}
		}
		CEngine::m_Scene->m_Texture.insert(pair<string, GLuint>(name, tex));
		node = node->NextSibling();
	}

	node = doc.FirstChildElement("config")->FirstChildElement("RenderPass");//m_companionWindowWidth
	while (node&&!strcmp(node->ToElement()->Value(), "RenderPass")) {
		CPass* pass;
		if (node->FirstChildElement("RenderMode")
			&& strcmp(node->FirstChildElement("RenderMode")->ToElement()->GetText(), "Deffer")) {
			pass = new CDifferPass();
		}
		else
			pass = new CPass();

		stream << node->FirstChildElement("Name")->GetText();
		string Name;
		stream >> Name;
		stream.clear();
		stream.str("");

		//Width 
		if (node->FirstChildElement("Width")) {
			pass->m_Width = atoi(node->FirstChildElement("Width")->GetText());
			pass->m_Height = atoi(node->FirstChildElement("Height")->GetText());
		}
		else {
			pass->m_Width = this->m_Scene->m_WindowWidth;
			pass->m_Height = this->m_Scene->m_WindowHeight;
		}

		//Model
		if(node->FirstChildElement("Mesh")) {
			const char* ModelPath = node->FirstChildElement("Mesh")->GetText();
			pass->m_Models.push_back(new CModel(ModelPath, Mesh));
		}

		if (node->FirstChildElement("PointCloud")) {
			const char* PointCloudPath = node->FirstChildElement("PointCloud")->GetText();
			pass->m_Models.push_back(new CModel(PointCloudPath, PointCloud));
		}

		if (node->FirstChildElement("Window")) {
			pass->m_Models.push_back(new CModel("", Window));

		}

		//Shader
		const char* VertFile = node->FirstChildElement("VertexShader")->GetText();
		const char* FragFile = node->FirstChildElement("FragmentShader")->GetText();
		if (!pass->setShader(VertFile, FragFile)) {
			std::cout << "Shader for " << Name << " init failed" << std::endl;
			return false;
		}

		glGenFramebuffers(1, &pass->m_FrameBuffer);

		int TargetCount = 0;
		glBindFramebuffer(GL_FRAMEBUFFER, pass->m_FrameBuffer);

		string OutTexture;
		XMLElement* Element = node->FirstChildElement("OutTexture");
		while (Element) {
			if(strcmp(Element->Name(),"OutTexture"))
				break;
			pass->m_IsTargetTexture = true;
			stream << Element->GetText();
			stream >> OutTexture;
			stream.clear();
			stream.str("");

			glFramebufferTexture2D(GL_FRAMEBUFFER, (GL_COLOR_ATTACHMENT0 + TargetCount), GL_TEXTURE_2D, CEngine::m_Scene->m_Texture.at(OutTexture), 0);

			++TargetCount;
			Element = Element->NextSiblingElement();
		}
		if (TargetCount > 0) {
			GLenum *Draws;
			Draws = (GLenum*)malloc(sizeof(GLenum)*TargetCount);
			for (int i = 0; i < TargetCount; i++)
				Draws[i] = GL_COLOR_ATTACHMENT0 + i;
			glDrawBuffers(TargetCount, Draws);
		}
		GLuint DepthRenderBuffer;
		glGenRenderbuffers(1, &DepthRenderBuffer);
		glBindRenderbuffer(GL_RENDERBUFFER, DepthRenderBuffer);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, pass->m_Width, pass->m_Height); // use a single renderbuffer object for both a depth AND stencil buffer.
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, DepthRenderBuffer); // now actually attach it
		//glBindRenderbuffer(GL_RENDERBUFFER, 0);
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
			cout << "FrameBuffer false" << endl;
			return false;
		}

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		CEngine::m_Scene->m_Pass.insert(pair<string, CPass*>(Name, pass));

		node = node->NextSibling();
		if (node == NULL)
			break;
	}

	return true;
}

bool CEngine::__initScene(string configFile) {
	CEngine::m_Scene = new CScene();
	stringstream stream;
	XMLDocument doc;
	XMLError errXml = doc.LoadFile(configFile.c_str());
	if (XML_SUCCESS == errXml) {
		XMLNode *node = 0;
		node = doc.FirstChildElement("config")->FirstChild();//m_companionWindowWidth
		if (!node) {
			std::cout << "Read XML File Failed" << std::endl;
			return false;
		}
		stream << node->ToElement()->GetText();
		stream >> CEngine::m_Scene->m_WindowWidth;
		stream.clear();
		stream.str("");
		node = node->NextSibling();
		stream << node->ToElement()->GetText();
		stream >> CEngine::m_Scene->m_WindowHeight;
		stream.clear();
		stream.str("");

		float CameraSpeed = 10.0f;
		node = doc.FirstChildElement("config")->FirstChildElement("cameraSpeed");
		if (node) {
			stream << node->ToElement()->GetText();
			stream >> CameraSpeed;
		}
		

		CEngine::m_lastX = CEngine::m_Scene->m_WindowWidth / 2.0f;
		CEngine::m_lastY = CEngine::m_Scene->m_WindowHeight / 2.0f;

		CEngine::m_Scene->m_Camera = new CCamera(CameraSpeed,glm::vec3(0, 0, 3));

		return true;
	} else {
		cout << "XML file can't open" << endl;
		return false;
	}
}

CScene* CEngine::m_Scene = NULL;
bool CEngine::m_firstMouse = true;
float CEngine::m_lastX = 0;
float CEngine::m_lastY = 0;
float CEngine::m_deltaTime = 0;
float CEngine::m_lastFrame = 0;
bool CEngine::m_mouseClicked = false;