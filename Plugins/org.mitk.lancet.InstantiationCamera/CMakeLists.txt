project(org_mitk_lancet_InstantiationCamera)

mitk_create_plugin(
  EXPORT_DIRECTIVE INSTANTIATIONCAMERA_EXPORT
  EXPORTED_INCLUDE_SUFFIXES src
  MODULE_DEPENDS MitkQtWidgetsExt MitkLancetHardwareDevice
)
