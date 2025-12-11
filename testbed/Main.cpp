#include "Core.h"
#include "Visualization.h"

using namespace nph;

// Global functions
/// The application entry point
int main()
{
	try
	{
		Visualization* visualization = Visualization::getInstance();
		if (visualization == nullptr)
		{
			return -1;
		}

		while (visualization->isRunning())
		{
			visualization->startFrame();
			ImGui::ShowDemoWindow();
			visualization->endFrame();
		}
		return 0;
	}
	catch (const std::exception& exception)
	{
		logError("Exception caught: ", exception.what());
	}
	catch (...)
	{
		logError("Unknown exception caught.");
	}
	return -1;
}
