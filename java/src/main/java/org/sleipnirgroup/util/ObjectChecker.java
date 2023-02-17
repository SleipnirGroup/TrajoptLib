// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

public class ObjectChecker {
    /**
     * Throws an exception if the list or any of its elements are null and
     * returns an unmodifiable copy of the given list.
     *
     * @param <E> the list type
     * @param list the list to wrap
     * @return the unmofifiable copy of {@code list}
     * @throws NullPointerException if the list or any of its elements are null
     */
    public static <E> List<E> requireNonNullAndWrapUnmodifiable(List<E> list, String listNullMessage, String itemNullMessage) throws NullPointerException {
        Objects.requireNonNull(list, listNullMessage);
        for (E item : list) {
            Objects.requireNonNull(item, itemNullMessage);
        }
        return Collections.unmodifiableList(new ArrayList<>(list));
    }

    public static int requireNonNegative(int value, String message) throws IllegalArgumentException {
        if (value < 0) {
            throw new IllegalArgumentException(message);
        }
        return value;
    }
}
