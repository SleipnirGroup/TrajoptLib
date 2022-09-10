#include <iostream>

template<typename T>
struct Base {
    T value;

    Base(T value) : value(value) {
    }
};

template<typename T>
struct Derived : Base<T> {
    Derived(T value) : Base<T>(value) {
    }
};

template<typename T>
struct DoubleD : Derived<T> {
    DoubleD(T value) : Derived<T>(value) {
    }

    void Print() {
        std::cout << Base<T>::value;
    }
};

int main() {
    DoubleD<int> derived(4);
    derived.Print();
    std::cout << derived.value << std::endl;
}