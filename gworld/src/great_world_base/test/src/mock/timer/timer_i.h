//
// Created by bondshi on 2020/5/22.
//
#pragma once
#include <assert.h>
#include <string.h>

#include <list>
#include <unordered_map>

#include "timer.h"

namespace gw {
namespace timer {

struct TimerNode {
  timerid_t id;
  unsigned interval;
  unsigned call_num;
  timestamp_t expire_time;
  bool repeatable;
  bool removed;
  TimerCb cb;
};

using TimerList = std::list<TimerNode *>;

#define MAX_LAYER_NUM 5  // 256 * 64 * 64 * 64 * 64
#define LAYER_ENTRY_NUM 64
#define SLOT_ENTRY_NUM 256

struct MultiLayerWheel {
  int layer;  // 0 is slots, > 0 is layers
  int child_num;
  MultiLayerWheel *parent;
  union {
    MultiLayerWheel *layers[LAYER_ENTRY_NUM];
    TimerList *slots[SLOT_ENTRY_NUM];
  };
};

typedef int WheelCursor[MAX_LAYER_NUM];

static inline void GetInsertPos(const WheelCursor &current, int ticks,
                                WheelCursor &pos) {
  static const int kLayerBits[] = {8, 6, 6, 6, 6};

  memcpy(pos, current, sizeof(pos));
  for (int i = 0; i < MAX_LAYER_NUM - 1; ++i) {
    int n = 1 << kLayerBits[i];
    pos[i] += ticks % n;
    ticks >>= kLayerBits[i];
    if (pos[i] >= n) {
      pos[i] -= n;
      pos[i + 1] += 1;

      if (ticks == 0 && pos[i + 1] < LAYER_ENTRY_NUM) break;
    }
  }

  if (pos[MAX_LAYER_NUM - 1] >= LAYER_ENTRY_NUM) {
    pos[MAX_LAYER_NUM - 1] %= LAYER_ENTRY_NUM;
  }
}

class TimerManagerImpl : public ATimerManager {
 public:
  TimerManagerImpl();
  virtual ~TimerManagerImpl();

  int Init() override;
  void Cleanup() override;
  int Update(int max_call_num) override;

  int CreateTimer(timerid_t timer_id, const TimerCb &cb, unsigned int interval_ms,
                  bool repeatable) override;
  void DeleteTimer(timerid_t timer_id) override;

  unsigned int GetTimerNum() const override {
    return (unsigned int)timers_.size();
  }

  TimerInfo *GetTimerInfo(timerid_t timer_id, TimerInfo *info) const override;

  void SpeedUp(timestamp_t t) override;

 private:
  void InsertTimer(TimerNode *node, int added_ticks) {
    WheelCursor pos;
    GetInsertPos(cursor_, node->interval / kTickUnit + added_ticks - 1, pos);

    TimerList *target_list = GetTimerList(pos, true);
    target_list->push_back(node);
    node->expire_time = GetTimeStamp() + node->interval;
  }

  void IncCursor() {
    last_time_ += kTickUnit;
    cursor_[0] = (cursor_[0] + 1) % SLOT_ENTRY_NUM;
    for (int i = 1; i < MAX_LAYER_NUM; ++i) {
      if (cursor_[i - 1] == 0) {
        cursor_[i] = (cursor_[i] + 1) % LAYER_ENTRY_NUM;
      } else {
        break;
      }
    }
  }

  MultiLayerWheel *GetLowestWheel(const WheelCursor &cursor, bool with_create) {
    MultiLayerWheel *w = &wheel_;
    int layer = w->layer;
    for (; layer > 0; layer--) {
      int p = cursor[layer];
      MultiLayerWheel *sw = w->layers[p];
      if (sw == NULL) {
        if (!with_create)
          return NULL;

        sw = new MultiLayerWheel;
        memset(sw, 0, sizeof(*sw));
        sw->layer = layer - 1;
        sw->parent = w;
        w->layers[p] = sw;
        w->child_num++;
      }

      w = sw;
    }

    return w;
  }

  TimerList *GetTimerList(MultiLayerWheel *w, int slot_no, bool with_create) {
    if (w == NULL)
      return NULL;

    assert(w->layer == 0);
    assert(slot_no >= 0 && slot_no < SLOT_ENTRY_NUM);

    if (w->slots[slot_no] == NULL && with_create) {
      w->slots[slot_no] = new TimerList;
      w->child_num++;
    }

    return w->slots[slot_no];
  }

  TimerList *GetTimerList(const WheelCursor &cursor, bool with_create) {
    MultiLayerWheel *w = GetLowestWheel(cursor, with_create);
    if (w == NULL)
      return NULL;

    return GetTimerList(w, cursor[0], with_create);
  }

  void FreeWheelSlot(MultiLayerWheel *w, const WheelCursor &cursor) {
    assert(w != NULL);
    int slot_no = cursor[w->layer];
    if (w->layer == 0) {
      w->slots[slot_no] = NULL;
    } else {
      w->layers[slot_no] = NULL;
    }

    w->child_num--;
    if (w->child_num > 0)
      return;

    assert(w->child_num == 0);
    if (w->layer == MAX_LAYER_NUM - 1) {
      assert(w == &wheel_);
      return;
    }

    MultiLayerWheel *p = w->parent;
    delete w;

    FreeWheelSlot(p, cursor);
  }

  void DeleteNode(TimerNode *node) {
    if (!node->removed) {
      timers_.erase(node->id);
    }
    delete node;
  }

  timestamp_t GetTimeStamp() const {
    return GetProcTickMs() + speed_up_ts_;
  }

  typedef std::unordered_map<timerid_t, TimerNode *> TimerMap;
  TimerMap timers_;
  MultiLayerWheel wheel_;
  WheelCursor cursor_;
  timestamp_t last_time_ = 0;
  timestamp_t speed_up_ts_ = 0;
};

}  // namespace timer
}  // namespace gw
