project(org_mitk_lancet_neurosurgery)

mitk_create_plugin(
  EXPORT_DIRECTIVE NEUROSURGERY_EXPORT
  EXPORTED_INCLUDE_SUFFIXES src
  MODULE_DEPENDS MitkQtWidgetsExt MitkIGTUI MitkIGT  MitkBoundingShape MitkRemeshing  MitkMatchPointRegistration MitkMatchPointRegistrationUI MitkGizmo
)
