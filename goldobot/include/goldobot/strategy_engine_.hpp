
#include <cstdint>

class StrategyEngine
{
public:
	StrategyEngine();
	void onMessage(uint16_t id, const char* data, size_t data_size);
	void sendMessage(uint16_t id, const char* data, size_t data_size);

private:

};