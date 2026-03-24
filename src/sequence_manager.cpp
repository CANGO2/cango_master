#include <sequence_manager.hpp>

namespace cango_master {


SequenceManager::SequenceManager() {
  this->setup();  }

SequenceManager::~SequenceManager() {}

void SequenceManager::change_status(enum task_name task_num, int &act) {
if task_num == task_name::LLM {
  now_status.use_llm = act;
}

}

SequenceManager::update_status() {
prev_status = update_status;
}

}
