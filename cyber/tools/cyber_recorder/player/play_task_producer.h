/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_PRODUCER_H_
#define CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_PRODUCER_H_

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "cyber/message/raw_message.h"
#include "cyber/node/node.h"
#include "cyber/node/writer.h"
#include "cyber/record/record_reader.h"
#include "cyber/tools/cyber_recorder/player/play_param.h"
#include "cyber/tools/cyber_recorder/player/play_task_buffer.h"

namespace apollo
{
namespace cyber
{
namespace record
{
class PlayTaskProducer
{
public:
    using NodePtr = std::shared_ptr<Node>;
    using ThreadPtr = std::unique_ptr<std::thread>;
    using TaskBufferPtr = std::shared_ptr<PlayTaskBuffer>;
    using RecordReaderPtr = std::shared_ptr<RecordReader>;
    using WriterPtr = std::shared_ptr<Writer<message::RawMessage>>;
    using WriterMap = std::unordered_map<std::string, WriterPtr>;
    using MessageTypeMap = std::unordered_map<std::string, std::string>;

    PlayTaskProducer(const TaskBufferPtr& task_buffer,
                     const PlayParam& play_param);
    virtual ~PlayTaskProducer();

    bool Init();
    void Start();
    void Stop();

    bool is_initialized() const { return is_initialized_.load(); }

    const PlayParam& play_param() const { return play_param_; }
    bool is_stopped() const { return is_stopped_.load(); }

    // processed_time，已经处理的时间
    int publish_play_info(double processed_time);

    /**
     * @brief Reset player producer for dv will repeatedly use it.
     * reset the start time when dv reset play record progress.
     */
    void Reset(const double& progress_time_s);

    /**
     * @brief Preload the player,producer fill play_task_buffer before
     * playing.
     */
    void FillPlayTaskBuffer();

private:
    bool ReadRecordInfo();
    bool UpdatePlayParam();
    bool CreateWriters();
    void ThreadFunc();

    PlayParam play_param_;
    TaskBufferPtr task_buffer_;
    ThreadPtr produce_th_;

    std::atomic<bool> is_initialized_;
    std::atomic<bool> is_stopped_;

    NodePtr node_;
    WriterMap writers_;
    MessageTypeMap msg_types_;
    std::vector<RecordReaderPtr> record_readers_;

    // RecordViewerPtr record_viewer_ptr_;

    uint64_t earliest_begin_time_;
    uint64_t latest_end_time_;
    uint64_t total_msg_num_;

    static const uint32_t kMinTaskBufferSize;
    static const uint32_t kPreloadTimeSec;
    static const uint64_t kSleepIntervalNanoSec;

    std::shared_ptr<apollo::cyber::Writer<apollo::cyber::proto::PlayInfo>>
            play_state_writer_;

    apollo::cyber::proto::PlayInfo play_info_;

    // 需要处理数据的总时间
    double total_progress_time_s_;

    double history_time_;
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_PLAYER_PLAY_TASK_PRODUCER_H_
