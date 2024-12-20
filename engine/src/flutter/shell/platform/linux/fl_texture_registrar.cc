// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_texture_registrar.h"

#include <gmodule.h>

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"
#include "flutter/shell/platform/linux/fl_pixel_buffer_texture_private.h"
#include "flutter/shell/platform/linux/fl_texture_gl_private.h"
#include "flutter/shell/platform/linux/fl_texture_private.h"
#include "flutter/shell/platform/linux/fl_texture_registrar_private.h"

G_DECLARE_FINAL_TYPE(FlTextureRegistrarImpl,
                     fl_texture_registrar_impl,
                     FL,
                     TEXTURE_REGISTRAR_IMPL,
                     GObject)

struct _FlTextureRegistrarImpl {
  GObject parent_instance;

  // Weak reference to the engine this texture registrar is created for.
  GWeakRef engine;

  // ID to assign to the next new texture.
  int64_t next_id;

  // Internal record for registered textures.
  //
  // It is a map from Flutter texture ID to #FlTexture instance created by
  // plugins.  The keys are directly stored int64s. The values are stored
  // pointer to #FlTexture.  This table is freed by the responder.
  GHashTable* textures;

  // The mutex guard to make `textures` thread-safe.
  GMutex textures_mutex;
};

static void fl_texture_registrar_impl_iface_init(
    FlTextureRegistrarInterface* iface);

G_DEFINE_INTERFACE(FlTextureRegistrar, fl_texture_registrar, G_TYPE_OBJECT)

G_DEFINE_TYPE_WITH_CODE(
    FlTextureRegistrarImpl,
    fl_texture_registrar_impl,
    G_TYPE_OBJECT,
    G_IMPLEMENT_INTERFACE(fl_texture_registrar_get_type(),
                          fl_texture_registrar_impl_iface_init))

static void fl_texture_registrar_default_init(
    FlTextureRegistrarInterface* iface) {}

static void fl_texture_registrar_impl_dispose(GObject* object) {
  FlTextureRegistrarImpl* self = FL_TEXTURE_REGISTRAR_IMPL(object);

  g_mutex_lock(&self->textures_mutex);
  g_clear_pointer(&self->textures, g_hash_table_unref);
  g_mutex_unlock(&self->textures_mutex);

  g_weak_ref_clear(&self->engine);
  g_mutex_clear(&self->textures_mutex);

  G_OBJECT_CLASS(fl_texture_registrar_impl_parent_class)->dispose(object);
}

static void fl_texture_registrar_impl_class_init(
    FlTextureRegistrarImplClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_texture_registrar_impl_dispose;
}

static gboolean register_texture(FlTextureRegistrar* registrar,
                                 FlTexture* texture) {
  FlTextureRegistrarImpl* self = FL_TEXTURE_REGISTRAR_IMPL(registrar);

  if (FL_IS_TEXTURE_GL(texture) || FL_IS_PIXEL_BUFFER_TEXTURE(texture)) {
    g_autoptr(FlEngine) engine = FL_ENGINE(g_weak_ref_get(&self->engine));
    if (engine == nullptr) {
      return FALSE;
    }

    // We ideally would use numeric IDs, but for backwards compatibility with
    // existing code use the address of the texture. Once all code uses
    // fl_texture_get_id we can re-enable this method. See
    // https://github.com/flutter/flutter/issues/124009 int64_t id =
    // self->next_id++;
    int64_t id = reinterpret_cast<int64_t>(texture);
    if (fl_engine_register_external_texture(engine, id)) {
      fl_texture_set_id(texture, id);
      g_mutex_lock(&self->textures_mutex);
      g_hash_table_insert(self->textures, GINT_TO_POINTER(id),
                          g_object_ref(texture));
      g_mutex_unlock(&self->textures_mutex);
      return TRUE;
    } else {
      return FALSE;
    }
  } else {
    // We currently only support #FlTextureGL and #FlPixelBufferTexture.
    return FALSE;
  }
}

static FlTexture* lookup_texture(FlTextureRegistrar* registrar,
                                 int64_t texture_id) {
  FlTextureRegistrarImpl* self = FL_TEXTURE_REGISTRAR_IMPL(registrar);
  g_mutex_lock(&self->textures_mutex);
  FlTexture* texture = reinterpret_cast<FlTexture*>(
      g_hash_table_lookup(self->textures, GINT_TO_POINTER(texture_id)));
  g_mutex_unlock(&self->textures_mutex);
  return texture;
}

static gboolean mark_texture_frame_available(FlTextureRegistrar* registrar,
                                             FlTexture* texture) {
  FlTextureRegistrarImpl* self = FL_TEXTURE_REGISTRAR_IMPL(registrar);

  g_autoptr(FlEngine) engine = FL_ENGINE(g_weak_ref_get(&self->engine));
  if (engine == nullptr) {
    return FALSE;
  }

  return fl_engine_mark_texture_frame_available(engine,
                                                fl_texture_get_id(texture));
}

static gboolean unregister_texture(FlTextureRegistrar* registrar,
                                   FlTexture* texture) {
  FlTextureRegistrarImpl* self = FL_TEXTURE_REGISTRAR_IMPL(registrar);

  g_autoptr(FlEngine) engine = FL_ENGINE(g_weak_ref_get(&self->engine));
  if (engine == nullptr) {
    return FALSE;
  }

  gboolean result =
      fl_engine_unregister_external_texture(engine, fl_texture_get_id(texture));

  g_mutex_lock(&self->textures_mutex);
  if (!g_hash_table_remove(self->textures,
                           GINT_TO_POINTER(fl_texture_get_id(texture)))) {
    g_warning("Unregistering a non-existent texture %p", texture);
  }
  g_mutex_unlock(&self->textures_mutex);

  return result;
}

static void shutdown(FlTextureRegistrar* registrar) {
  FlTextureRegistrarImpl* self = FL_TEXTURE_REGISTRAR_IMPL(registrar);

  // Unregister any textures.
  g_mutex_lock(&self->textures_mutex);
  g_autoptr(GHashTable) textures = self->textures;
  self->textures = g_hash_table_new_full(g_direct_hash, g_direct_equal, nullptr,
                                         g_object_unref);
  g_hash_table_remove_all(textures);
  g_mutex_unlock(&self->textures_mutex);
}

static void fl_texture_registrar_impl_iface_init(
    FlTextureRegistrarInterface* iface) {
  iface->register_texture = register_texture;
  iface->lookup_texture = lookup_texture;
  iface->mark_texture_frame_available = mark_texture_frame_available;
  iface->unregister_texture = unregister_texture;
  iface->shutdown = shutdown;
}

static void fl_texture_registrar_impl_init(FlTextureRegistrarImpl* self) {
  self->next_id = 1;
  self->textures = g_hash_table_new_full(g_direct_hash, g_direct_equal, nullptr,
                                         g_object_unref);
  // Initialize the mutex for textures.
  g_mutex_init(&self->textures_mutex);
}

G_MODULE_EXPORT gboolean fl_texture_registrar_register_texture(
    FlTextureRegistrar* self,
    FlTexture* texture) {
  g_return_val_if_fail(FL_IS_TEXTURE_REGISTRAR(self), FALSE);
  g_return_val_if_fail(FL_IS_TEXTURE(texture), FALSE);

  return FL_TEXTURE_REGISTRAR_GET_IFACE(self)->register_texture(self, texture);
}

FlTexture* fl_texture_registrar_lookup_texture(FlTextureRegistrar* self,
                                               int64_t texture_id) {
  g_return_val_if_fail(FL_IS_TEXTURE_REGISTRAR(self), NULL);

  return FL_TEXTURE_REGISTRAR_GET_IFACE(self)->lookup_texture(self, texture_id);
}

G_MODULE_EXPORT gboolean fl_texture_registrar_mark_texture_frame_available(
    FlTextureRegistrar* self,
    FlTexture* texture) {
  g_return_val_if_fail(FL_IS_TEXTURE_REGISTRAR(self), FALSE);

  return FL_TEXTURE_REGISTRAR_GET_IFACE(self)->mark_texture_frame_available(
      self, texture);
}

G_MODULE_EXPORT gboolean fl_texture_registrar_unregister_texture(
    FlTextureRegistrar* self,
    FlTexture* texture) {
  g_return_val_if_fail(FL_IS_TEXTURE_REGISTRAR(self), FALSE);

  return FL_TEXTURE_REGISTRAR_GET_IFACE(self)->unregister_texture(self,
                                                                  texture);
}

void fl_texture_registrar_shutdown(FlTextureRegistrar* self) {
  g_return_if_fail(FL_IS_TEXTURE_REGISTRAR(self));

  return FL_TEXTURE_REGISTRAR_GET_IFACE(self)->shutdown(self);
}

FlTextureRegistrar* fl_texture_registrar_new(FlEngine* engine) {
  FlTextureRegistrarImpl* self = FL_TEXTURE_REGISTRAR_IMPL(
      g_object_new(fl_texture_registrar_impl_get_type(), nullptr));

  // Added to stop compiler complaining about an unused function.
  FL_IS_TEXTURE_REGISTRAR_IMPL(self);

  g_weak_ref_init(&self->engine, G_OBJECT(engine));

  return FL_TEXTURE_REGISTRAR(self);
}
