/* Copyright 2016 Esteve Fernandez <esteve@apache.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.ros2.rcljava;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import org.ros2.rcljava.interfaces.MessageDefinition;

/**
 * {@inheritDoc}
 */
public class PublisherImpl<T extends MessageDefinition> implements Publisher<T> {

  private static final Logger logger = LoggerFactory.getLogger(PublisherImpl.class);

  static {
    try {
      System.loadLibrary("rcljavaPublisherImpl__" + RCLJava.getRMWIdentifier());
    } catch (UnsatisfiedLinkError ule) {
      logger.error("Native code library failed to load.\n" + ule);
      System.exit(1);
    }
  }

  /**
   * An integer that represents a pointer to the underlying ROS2 node
   * structure (rcl_node_t).
   */
  private final long nodeHandle;

  /**
   * An integer that represents a pointer to the underlying ROS2 publisher
   * structure (rcl_publisher_t).
   */
  private final long publisherHandle;

  /**
   * The topic to which this publisher will publish messages.
   */
  private final String topic;

  /**
   * Constructor.
   *
   * @param nodeHandle A pointer to the underlying ROS2 node structure that
   *     created this subscription, as an integer. Must not be zero.
   * @param publisherHandle A pointer to the underlying ROS2 publisher
   *     structure, as an integer. Must not be zero.
   * @param topic The topic to which this publisher will publish messages.
   */
  public PublisherImpl(final long nodeHandle, final long publisherHandle,
      final String topic) {
    this.nodeHandle = nodeHandle;
    this.publisherHandle = publisherHandle;
    this.topic = topic;
  }

  /**
   * Publish a message via the underlying ROS2 mechanisms.
   *
   * @param <T> The type of the messages that this publisher will publish.
   * @param publisherHandle A pointer to the underlying ROS2 publisher
   *     structure, as an integer. Must not be zero.
   * @param message An instance of the &lt;T&gt; parameter.
   */
  private static native <T extends MessageDefinition> void nativePublish(
      long publisherHandle, T message);

  /**
   * {@inheritDoc}
   */
  public final void publish(final T message) {
    nativePublish(this.publisherHandle, message);
  }

  /**
   * Destroy a ROS2 publisher (rcl_publisher_t).
   *
   * @param nodeHandle A pointer to the underlying ROS2 node structure that
   *     created this subscription, as an integer. Must not be zero.
   * @param publisherHandle A pointer to the underlying ROS2 publisher
   *     structure, as an integer. Must not be zero.
   */
  private static native void nativeDispose(
      long nodeHandle, long publisherHandle);

  /**
   * {@inheritDoc}
   */
  public final void dispose() {
    nativeDispose(this.nodeHandle, this.publisherHandle);
  }

  /**
   * {@inheritDoc}
   */
  public final long getNodeHandle() {
    return this.nodeHandle;
  }

  /**
   * {@inheritDoc}
   */
  public final long getPublisherHandle() {
    return this.publisherHandle;
  }
}
