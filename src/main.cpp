#include "implot.h" // NOLINT
#include "imgui_impl_glfw.h" // NOLINT
#include "imgui_impl_opengl3.h" // NOLINT
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <string>
#include <memory>
#include <utility>
#include "quickplot/application.hpp"

static void glfw_error_callback(int error, const char * description)
{
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

int main(int argc, char ** argv)
{
  auto non_ros_args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  auto node = std::make_shared<quickplot::QuickPlotNode>();
  std::thread ros_thread([ = ] {
      rclcpp::spin(node);
    });

  fs::path config_file;
  if (non_ros_args.size() > 1) {
    // non ROS arguments after the program name are interpreted as config file paths
    config_file = non_ros_args[1];
  } else {
    config_file = quickplot::get_default_config_path();
  }
  quickplot::Application app(config_file, node);

  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit()) {
    return EXIT_FAILURE;
  }

  const char * glsl_version = "#version 150";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_SAMPLES, 8);

  GLFWwindow * window = glfwCreateWindow(1280, 720, "quickplot", NULL, NULL);
  if (window == NULL) {
    return EXIT_FAILURE;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);   // enable vsync

  bool err = glewInit() != GLEW_OK;
  if (err) {
    fprintf(stderr, "Failed to initialize OpenGL loader!\n");
    return EXIT_FAILURE;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGuiIO & io = ImGui::GetIO();
  io.ConfigFlags &= ~(ImGuiConfigFlags_ViewportsEnable);
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

  auto imgui_ini_path = quickplot::get_default_config_directory().append("imgui.ini").string();
  io.IniFilename = imgui_ini_path.c_str();

  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  ImVec4 clear_color = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];

  while (rclcpp::ok()) {
    if (glfwWindowShouldClose(window)) {
      // ensure ROS finishes up if window is closed
      rclcpp::shutdown();
      break;
    }
    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    auto dock_id = ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());
    ImGui::SetNextWindowDockID(dock_id);
    app.update();
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(
      clear_color.x * clear_color.w, clear_color.y * clear_color.w,
      clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  // save configuration before closing
  app.save_application_config();
  ros_thread.join();
  return EXIT_SUCCESS;
}
