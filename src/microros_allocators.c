#include "./microros_allocators.h"

#include "./config.h"

#include <stdint.h>

static uint8_t heap[HEAP_SIZE];
static size_t current_pointer = 0;

uint32_t max_used_heap()
{
  return current_pointer;
}

void assert_position()
{
  if (current_pointer >= sizeof(heap)) {
    while (1) {}
  }
}

void * custom_allocate(size_t size, void * state)
{
  if (size % 4 != 0) {
    size += 4 - (size % 4);
  }

  size_t p = current_pointer;

  current_pointer += size;

  assert_position();
  return (void *) &heap[p];
}

void custom_deallocate(void * pointer, void * state)
{
  (void) state;
  (void) pointer;
}

void * custom_reallocate(void * pointer, size_t size, void * state)
{
  if (size % 4 != 0) {
    size += 4 - (size % 4);
  }

  size_t p = current_pointer;

  current_pointer += size;

  assert_position();
  return (void *) &heap[p];
}

void * custom_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state)
{
  size_t size = number_of_elements * size_of_element;

  if (size % 4 != 0) {
    size += 4 - (size % 4);
  }

  size_t p = current_pointer;

  current_pointer += size;

  assert_position();
  return (void *) &heap[p];
}
