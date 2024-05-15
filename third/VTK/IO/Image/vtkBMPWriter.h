// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause
/**
 * @class   vtkBMPWriter
 * @brief   Writes Windows BMP files.
 *
 * vtkBMPWriter writes BMP files. The data type
 * of the file is unsigned char regardless of the input type.
 *
 * @sa
 * vtkBMPReader
 */

#ifndef vtkBMPWriter_h
#define vtkBMPWriter_h

#include "vtkIOImageModule.h" // For export macro
#include "vtkImageWriter.h"

VTK_ABI_NAMESPACE_BEGIN
class vtkUnsignedCharArray;

class VTKIOIMAGE_EXPORT vtkBMPWriter : public vtkImageWriter
{
public:
  static vtkBMPWriter* New();
  vtkTypeMacro(vtkBMPWriter, vtkImageWriter);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /**
   * Write the image to memory (a vtkUnsignedCharArray)
   */
  vtkSetMacro(WriteToMemory, vtkTypeUBool);
  vtkGetMacro(WriteToMemory, vtkTypeUBool);
  vtkBooleanMacro(WriteToMemory, vtkTypeUBool);
  ///@}

  ///@{
  /**
   * When writing to memory this is the result, it will be NULL until the
   * data is written the first time
   */
  virtual void SetResult(vtkUnsignedCharArray*);
  vtkGetObjectMacro(Result, vtkUnsignedCharArray);
  ///@}

protected:
  vtkBMPWriter();
  ~vtkBMPWriter() override;

  void WriteFile(ostream* file, vtkImageData* data, int ext[6], int wExt[6]) override;
  void WriteFileHeader(ostream*, vtkImageData*, int wExt[6]) override;
  void MemoryWrite(int, vtkImageData*, int wExt[6], vtkInformation* inInfo) override;

private:
  vtkBMPWriter(const vtkBMPWriter&) = delete;
  void operator=(const vtkBMPWriter&) = delete;

  vtkUnsignedCharArray* Result;
};

VTK_ABI_NAMESPACE_END
#endif
