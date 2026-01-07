#include <sentinel/protocol/arguments.hpp>

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef __unix__
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

struct Child {
    std::string name;
    std::filesystem::path executable;
    std::vector<std::string> arguments;
#ifdef __unix__
    pid_t pid{-1};
#endif
};

void require_executable(const std::filesystem::path& path) {
    if (!std::filesystem::is_regular_file(path)) {
        throw std::invalid_argument("child executable does not exist");
    }
}

#ifdef __unix__
pid_t launch(const Child& child) {
    const auto pid = fork();
    if (pid < 0) {
        throw std::runtime_error("failed to fork child");
    }
    if (pid == 0) {
        std::vector<char*> values;
        values.push_back(const_cast<char*>(child.executable.c_str()));
        for (const auto& argument : child.arguments) {
            values.push_back(const_cast<char*>(argument.c_str()));
        }
        values.push_back(nullptr);
        execv(child.executable.c_str(), values.data());
        _exit(127);
    }
    return pid;
}

int wait_for(pid_t pid) {
    int status{};
    while (waitpid(pid, &status, 0) < 0) {
    }
    if (WIFEXITED(status)) {
        return WEXITSTATUS(status);
    }
    return 128 + WTERMSIG(status);
}
#endif

}

int main(int argc, char* argv[]) {
    try {
        if (argc == 2 && std::string_view(argv[1]) == "--help") {
            std::cout << "usage: sentinel_supervisor --sim PATH --agent PATH --scenario PATH --agent-id ID\n";
            return 0;
        }
        const auto simulator = sentinel::protocol::required_option(argc, argv, "--sim");
        const auto agent = sentinel::protocol::required_option(argc, argv, "--agent");
        const auto scenario = sentinel::protocol::required_option(argc, argv, "--scenario");
        const auto agent_id = sentinel::protocol::required_option(argc, argv, "--agent-id");
        require_executable(simulator);
        require_executable(agent);
#ifdef __unix__
        Child sim{"simulator", std::filesystem::path(simulator), {"--scenario", scenario}};
        Child worker{"agent", std::filesystem::path(agent), {"--agent-id", agent_id}};
        sim.pid = launch(sim);
        worker.pid = launch(worker);
        const auto simulator_status = wait_for(sim.pid);
        if (kill(worker.pid, SIGTERM) == 0) {
            wait_for(worker.pid);
        }
        return simulator_status;
#else
        throw std::runtime_error("process supervision is only available on Unix");
#endif
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
