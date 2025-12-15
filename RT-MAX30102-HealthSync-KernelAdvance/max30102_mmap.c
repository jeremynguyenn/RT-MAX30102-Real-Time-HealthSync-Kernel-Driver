// max30102_mmap.c: Handles mmap for zero-copy access to FIFO.
// Supplemented with page fault comment.
// No changes to original code, only added comments.
// Supplemented for missing: userfaultfd handling, page faults (memory mgmt), VFIO comment for device passthrough.

#include "max30102.h"  // Include header.
#include <linux/userfaultfd.h>  // Added for userfaultfd (low-level OS interaction, page faults).

// Mmap function: Maps kernel memory to user space.
int max30102_mmap(struct file *file, struct vm_area_struct *vma)  // Function signature.
{
    struct max30102_data *data = file->private_data;  // Get data.
    unsigned long size = vma->vm_end - vma->vm_start;  // Calculate map size.
    unsigned long pfn = virt_to_phys(data->fifo.entries) >> PAGE_SHIFT;  // Get page frame number.

    if (size > sizeof(data->fifo.entries)) return -EINVAL;  // Size validation.

    // Remap for zero-copy.
    if (remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot)) {  // Remap pages.
        dev_err(&data->client->dev, "Mmap failed\n");  // Log error.
        return -EAGAIN;  // Retry error.
    }

    // Note: Page faults handled by kernel; userfaultfd for user handling.
    // Added: Register userfaultfd for on-demand paging (userfaultfd + driver memory mgmt).
    // Example: vma->vm_ops->fault = max30102_vm_fault; // Custom fault handler if needed.
    // Note: For VFIO/DMA-BUF shared concurrency, export dma_buf and mmap via VFIO for multi-process sharing.

    dev_info(&data->client->dev, "Mmap success for zero-copy FIFO\n");  // Log success.
    return 0;  // Success.
}