#pragma once

#include <iostream>
#include <mutex>
#include <unordered_map>

namespace utilities {

template <typename Object>
class Singleton {
 protected:
  Singleton() = default;
  virtual ~Singleton() = default;

  Singleton(const Singleton&) = delete;
  Singleton(Singleton&&) = delete;
  Singleton& operator=(const Singleton&) = delete;

 public:
  static Object* GetInstance(bool create_flag = true) {
    static std::once_flag once_flag;
    if (instance_ == nullptr && create_flag) {
      std::call_once(once_flag, []() {
        instance_ = new Object();
        static _GR gt;
      });
    }

    return instance_;
  }

 protected:
  class _GR {
   public:
    ~_GR() {
      if (Singleton::instance_ != nullptr) {
        delete Singleton::instance_;
        Singleton::instance_ = nullptr;
      }
    }
  };

 private:
  static Object* instance_;
};

template <typename Object>
Object* Singleton<Object>::instance_ = nullptr;

template <typename Key, typename Object>
class SingletonHolder {
 private:
  SingletonHolder() {}

 public:
  ~SingletonHolder() {}
  static SingletonHolder* GetInstance() {
    static std::once_flag once_flag;
    if (instance_ == nullptr) {
      std::call_once(once_flag, []() {
        instance_ = new SingletonHolder();
        static _GR gt;
      });
    }

    return instance_;
  }

  Object& operator[](const Key& key) { return holder_[key]; }

  Object& get(const Key& key) { return holder_[key]; }

  void add(const Key& key, Object& obj) { holder_[key] = obj; }

  void add(const Key& key, Object&& obj) { holder_.emplace(key, obj); }

  void remove(const Key& key) { holder_.erase(key); }

 protected:
  class _GR {
   public:
    ~_GR() {
      if (SingletonHolder::instance_ != nullptr) {
        delete SingletonHolder::instance_;
        SingletonHolder::instance_ = nullptr;
      }
    }
  };

 private:
  static SingletonHolder* instance_;

  std::unordered_map<Key, Object> holder_;
};

template <typename Key, typename Object>
SingletonHolder<Key, Object>* SingletonHolder<Key, Object>::instance_ = nullptr;

}  // namespace utilities
