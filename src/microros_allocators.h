#ifndef MICROROS_ALLOCATORS_H_
#define MICROROS_ALLOCATORS_H_

#include <stdlib.h>
#include <stdint.h>

uint32_t max_used_heap();

void * custom_allocate(size_t size, void * state);
void custom_deallocate(void * pointer, void * state);
void * custom_reallocate(void * pointer, size_t size, void * state);
void * custom_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

#endif  // MICROROS_ALLOCATORS_H_
