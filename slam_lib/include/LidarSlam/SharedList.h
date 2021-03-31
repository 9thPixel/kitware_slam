//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2021-04-30
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//==============================================================================

#include <condition_variable>
#include <queue>
#include <shared_mutex>
#include <thread>

namespace LidarSlam
{
/**
 * @brief Shared queue class : FIFO container whith a shared
 * mutex and a conditional variable to allow access to
 * multiple threads in reading / writing / waiting for data
 */
template <typename T>
class SharedList {
public:

  // -------------Constructors--------------
  SharedList() = default;

  SharedList(int sizeMax)
  {
    this->SetSizeMax(sizeMax);
  }

  // -------------Start/Stop--------------
  // Clear all stored data
  void Reset()
  {
    std::unique_lock<std::shared_timed_mutex> lock(this->Mutex);
    this->RequestEnd = false;
    this->Data.clear();
  }

  // Free all waiting threads
  void Stop()
  {
    std::unique_lock<std::shared_timed_mutex> lock(this->Mutex);
    this->RequestEnd = true;
    this->Cond.notify_all();
  }

  // -------------Structure features---------------
  // Setter for SizeMax parameter,
  // remove oldest elements if current data size is greater than SizeMax
  void SetSizeMax(unsigned int m)
  { 
    std::unique_lock<std::shared_timed_mutex> lock(this->Mutex);
    this->SizeMax = m;
    while (this->Data.size() > m)
      this->Data.pop_front();
  }

  int GetSizeMax() const
  {
    std::shared_lock<std::shared_timed_mutex> lock(this->Mutex);
    return this->SizeMax;
  }

  int Size()
  {
    std::unique_lock<std::shared_timed_mutex> lock(this->Mutex);
    return this->Data.size();
  }

  // -------------Get data---------------

  // Get element with index
  T operator[](int index)
  {
    std::shared_lock<std::shared_timed_mutex> lock(this->Mutex);
    return Data[index];
  }

  // Get a copy of current data 
  std::deque<T> Copy() const
  {
    std::shared_lock<std::shared_timed_mutex> lock(this->Mutex);
    return this->Data;
  }

  // Copy last n elements in a vector
  // If there is not enough data, the output vector will be smaller than the queried number
  // If no input number, all list is returned
  std::vector<T> LastElements(int nQueries = -1) const
  {
    std::shared_lock<std::shared_timed_mutex> lock(this->Mutex);
    int s = this->Data.size();
    int nElements = nQueries >= 0 ? std::min(nQueries, s) : s;
    // Get iterator of oldest queried element
    auto firstEl = this->Data.end();
    for (int n = 0; n < nElements; ++n)
      --firstEl;
    return std::vector<T>(firstEl, this->Data.end());
  }

  // Get last element and store it in input value
  // Return true if last element exists and false if it doesn't
  bool Back(T& value) const
  {
    std::shared_lock<std::shared_timed_mutex> lock(this->Mutex);
    if (this->Data.empty())
      return false;
    value = this->Data.back();
    return true;
  }

  // Get first element
  // Return true if first element exists and false if it doesn't
  bool Front(T& value) const
  {
    std::shared_lock<std::shared_timed_mutex> lock(this->Mutex);
    if (this->Data.empty())
      return false;
    value = this->Data.front();
    return true;
  }

  // Wait for data and take out the oldest element
  // The thread calling this method will be blocked until data is added
  bool WaitAndPopFront(T& front) 
  {
    std::unique_lock<std::shared_timed_mutex> lock(this->Mutex);
    // The predicate protects from spurious notify 
    // and allows to lock mutex even if the notify has been
    // sent before getting to wait function.
    // No max time is added because of user interface handling
    this->Cond.wait(lock, [this] { return !this->Data.empty() || this->RequestEnd; });

    // If end was requested, return false
    if (this->Data.empty())
      return false;

    front = std::move(this->Data.front());
    this->Data.pop_front();
    return true;
  }

  // -------------Add elements---------------
  
  // Add the element value to the data and notify depending threads
  // Remove oldest element if the size is greater than max size.
  void EmplaceBack(const T& value)
  {
    std::unique_lock<std::shared_timed_mutex> lock(this->Mutex);
    this->Data.emplace_back(value);
    if (this->Data.size() > this->SizeMax)
      this->Data.pop_front();
    this->Cond.notify_one();
  }

private:
  // NOTE: shared_timed_mutex should be replaced by shared_mutex
  // when upgrading to C++17 
  mutable std::shared_timed_mutex Mutex;
  mutable std::condition_variable_any Cond;
  unsigned int SizeMax = 10;
  std::deque<T> Data;
  bool RequestEnd = false;
};

} // end of LidarSlam namespace