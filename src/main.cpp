#include "implot.h" // NOLINT
#include "imgui_impl_glfw.h" // NOLINT
#include "imgui_impl_opengl3.h" // NOLINT
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <string>
#include <memory>
#include <utility>
#include <filesystem>
#include "quickplot/application.hpp"
#include "quickplot/config.hpp"

static void glfw_error_callback(int error, const char * description)
{
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void print_exception_recursive(const std::exception & e, int level = 0, int after_level = 0)
{
  if (level >= after_level) {
    std::cerr << std::string(level, ' ') << e.what() << '\n';
  }
  try {
    std::rethrow_if_nested(e);
  } catch (const std::exception & e) {
    print_exception_recursive(e, level + 1, after_level);
  } catch (...) {
  }
}

static bool first_time = true;

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

  quickplot::ApplicationConfig config;
  try {
    config = quickplot::load_config(config_file);
    std::cout << "Using configuration at " << config_file << std::endl;
  } catch (const quickplot::config_error & e) {
    std::cerr << "Failed to read configuration from '" << config_file.c_str() << "'" << std::endl;
    print_exception_recursive(e, 0, 1);
    std::cout << "Using default configuration" << std::endl;
    config = quickplot::default_config();
  }

  quickplot::Application app(node);
  app.apply_config(config);

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

  GLFWwindow * window = glfwCreateWindow(1280, 480 * config.plots.size(), "quickplot", NULL, NULL);
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

  double font_du_point_size = 12.0; // TODO(ZeilingerM) get from os config
  double screen_dpi = 96.0; // TODO(ZeilingerM) get from monitor
  double point_pixel_size = screen_dpi / 72;
  double du_pixel_size = font_du_point_size * point_pixel_size;

  // Ubuntu standard font, TODO(ZeilingerM) fallback to default font
  io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf", du_pixel_size);

  auto imgui_ini_path = quickplot::get_default_config_directory().append("imgui.ini").string();
  io.IniFilename = imgui_ini_path.c_str();

  ImGui::StyleColorsLight();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  ImGuiStyle & style = ImGui::GetStyle();
  ImVec4 clear_color = style.Colors[ImGuiCol_WindowBg];

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

    // TODO(ZeilingerM) should probably be user-configured
    ImPlot::GetStyle().LineWeight = 2.0;

    static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_PassthruCentralNode;
    // build parent window with dock spaces
    ImGuiWindowFlags parent_window_flags = ImGuiWindowFlags_NoSavedSettings;
    parent_window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
      ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    parent_window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
    if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode) {
      parent_window_flags |= ImGuiWindowFlags_NoBackground;
    }

    ImGuiViewport * viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->Pos);
    ImGui::SetNextWindowSize(viewport->Size);
    ImGui::SetNextWindowViewport(viewport->ID);

    // remove redundant padding from outer window
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0, 0.0));
    ImGui::Begin("ParentDock", nullptr, parent_window_flags);
    ImGui::PopStyleVar();

    ImGuiID dockspace_id = ImGui::GetID("ParentDockSpace");
    ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);

    if (first_time) {
      first_time = false;
      ImGui::DockBuilderRemoveNode(dockspace_id); // clear any previous layout
      ImGui::DockBuilderAddNode(dockspace_id, dockspace_flags | ImGuiDockNodeFlags_DockSpace);
      ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->Size);

      float default_list_width = (1 - (1.0 / 1.61803398875)) / 2.0;
      ImGuiID dock_id_plot;
      ImGuiID dock_id_list = ImGui::DockBuilderSplitNode(
        dockspace_id, ImGuiDir_Left,
        default_list_width, nullptr, &dock_id_plot);
      // disable docking and bar in list window
      ImGui::DockBuilderGetNode(dock_id_list)->SetLocalFlags(
        ImGuiDockNodeFlags_NoDocking | ImGuiDockNodeFlags_NoTabBar);

      // we now dock our windows into the docking node we made above
      ImGui::DockBuilderDockWindow(quickplot::TOPIC_LIST_WINDOW_ID, dock_id_list);

      if (config.plots.size() >= 1) {
        ImGui::DockBuilderDockWindow("plot0", dock_id_plot);
        float equal_ratio = 1.0 / config.plots.size();
        for (size_t i = 1; i < config.plots.size(); i++) {
          ImGui::DockBuilderSplitNode(
            dock_id_plot, ImGuiDir_Down,
            equal_ratio, &dock_id_plot, nullptr);
          std::string win_id = "plot" + std::to_string(i);
          ImGui::DockBuilderDockWindow(win_id.c_str(), dock_id_plot);
        }
      }
      ImGui::DockBuilderFinish(dockspace_id);
    }

    ImGui::End();

    // update quickplot
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
  if (config_file.has_parent_path()) {
    fs::create_directories(config_file.parent_path());
  }
  quickplot::save_config(app.get_config(), config_file);

  ros_thread.join();
  return EXIT_SUCCESS;
}
